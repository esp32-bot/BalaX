#include "lan_server.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_http_server.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "miniz.h"  // ESP32 Mask ROM tinfl_decompress (0 bytes Flash!)
#include "nvs.h"
#include "prog_vm.h"
#include "sha256_hmac.h"
#include "spi_flash_mmap.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "status.h"
#include "wifi.h"

namespace lan_server {
namespace {

constexpr const char* TAG = "lan_server";
constexpr size_t NONCE_BYTES = 16;
constexpr uint64_t NONCE_TTL_US = 60ULL * 1000 * 1000;
constexpr size_t LOG_RING_BYTES = 4096;
constexpr BaseType_t HTTPD_CORE = 0;  // see start()
constexpr UBaseType_t MDNS_PRIORITY = 3;
constexpr size_t SHARED_HTTP_BYTES = 4096;
constexpr size_t SECTOR_BYTES = 4096;

StreamPullCallback stream_pull_cb = nullptr;
std::array<uint8_t, 32> lan_key{};
bool lan_key_set = false;

// 4-slot one-time-use nonce ring so concurrent clients (balax-remote + bx lan-cmd + phone)
// never clobber each other's pipelined X-Next-Nonce tokens (~112 B RAM total).
constexpr size_t NONCE_SLOTS = 4;
struct NonceSlot {
    std::array<uint8_t, NONCE_BYTES> bytes{};
    uint64_t issued_us = 0;
    bool valid = false;
};
portMUX_TYPE nonce_lock = portMUX_INITIALIZER_UNLOCKED;
NonceSlot nonce_ring[NONCE_SLOTS]{};
size_t nonce_next_slot = 0;

portMUX_TYPE log_lock = portMUX_INITIALIZER_UNLOCKED;
char log_ring[LOG_RING_BYTES];
uint32_t log_write_seq = 0;
vprintf_like_t prev_vprintf = nullptr;

// Single shared 4 KB buffer for single-threaded httpd handlers (cmd / stream / prog JSON).
// Replaces 32.8 KB of separate static buffers in the old ota.cpp!
uint8_t shared_http_buf[SHARED_HTTP_BYTES];

// Persistent flash wear counters (loaded from NVS on start, updated once per OTA).
uint32_t g_flash_erases_total = 0;
uint32_t g_last_sectors_erased = 0;
uint32_t g_last_sectors_skipped = 0;
bool g_has_factory = false;

void load_flash_wear_nvs() {
    nvs_handle_t h = 0;
    if (nvs_open("balax_ota", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "erases", &g_flash_erases_total);
        nvs_get_u32(h, "last_er", &g_last_sectors_erased);
        nvs_get_u32(h, "last_sk", &g_last_sectors_skipped);
        nvs_close(h);
    }
    g_has_factory = (esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr) != nullptr);
}

void save_flash_wear_nvs(uint32_t erased, uint32_t skipped) {
    g_last_sectors_erased = erased;
    g_last_sectors_skipped = skipped;
    g_flash_erases_total += erased;
    nvs_handle_t h = 0;
    if (nvs_open("balax_ota", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "erases", g_flash_erases_total);
        nvs_set_u32(h, "last_er", g_last_sectors_erased);
        nvs_set_u32(h, "last_sk", g_last_sectors_skipped);
        nvs_commit(h);
        nvs_close(h);
    }
}

// The ring is guarded by a spinlock, which also masks interrupts on this core, and core 0's
// tick interrupt is what wakes the balance loop on core 1: so copy in at most two memcpy()s
// (tens of microseconds for the whole ring at 80 MHz) rather than a byte at a time.
void copy_out_of_ring(uint32_t seq, char* out, size_t len) {
    const size_t at = seq % LOG_RING_BYTES;
    const size_t first = std::min(len, LOG_RING_BYTES - at);
    std::memcpy(out, log_ring + at, first);
    std::memcpy(out + first, log_ring, len - first);
}

void append_to_log_ring(const char* data, size_t len) {
    if (!data || len == 0) return;
    const size_t skip = len > LOG_RING_BYTES ? len - LOG_RING_BYTES : 0;  // only the end survives anyway
    const size_t keep = len - skip;
    taskENTER_CRITICAL(&log_lock);
    const size_t at = (log_write_seq + skip) % LOG_RING_BYTES;
    const size_t first = std::min(keep, LOG_RING_BYTES - at);
    std::memcpy(log_ring + at, data + skip, first);
    std::memcpy(log_ring, data + skip + first, keep - first);
    log_write_seq += len;
    taskEXIT_CRITICAL(&log_lock);
}

int tee_vprintf(const char* fmt, va_list args) {
    char tmp[256];
    va_list copy;
    va_copy(copy, args);
    const int n = vsnprintf(tmp, sizeof tmp, fmt, copy);
    va_end(copy);
    if (n > 0) {
        append_to_log_ring(tmp, std::min<size_t>(static_cast<size_t>(n), sizeof tmp - 1));
    }
    return prev_vprintf ? prev_vprintf(fmt, args) : vprintf(fmt, args);
}

void issue_nonce(std::array<uint8_t, NONCE_BYTES>& out_nonce) {
    esp_fill_random(out_nonce.data(), out_nonce.size());
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    taskENTER_CRITICAL(&nonce_lock);
    // Prefer an already-consumed/expired slot; otherwise advance ring cursor.
    size_t idx = nonce_next_slot;
    for (size_t i = 0; i < NONCE_SLOTS; ++i) {
        const size_t cand = (nonce_next_slot + i) % NONCE_SLOTS;
        if (!nonce_ring[cand].valid || (now - nonce_ring[cand].issued_us) > NONCE_TTL_US) {
            idx = cand;
            break;
        }
    }
    nonce_ring[idx].bytes = out_nonce;
    nonce_ring[idx].issued_us = now;
    nonce_ring[idx].valid = true;
    nonce_next_slot = (idx + 1) % NONCE_SLOTS;
    taskEXIT_CRITICAL(&nonce_lock);
}

bool consume_nonce(const char* nonce_hex, std::array<uint8_t, NONCE_BYTES>& out_nonce) {
    if (!crypto::from_hex(nonce_hex, out_nonce.data(), NONCE_BYTES)) {
        return false;
    }
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    bool ok = false;
    taskENTER_CRITICAL(&nonce_lock);
    for (size_t i = 0; i < NONCE_SLOTS; ++i) {
        if (nonce_ring[i].valid && (now - nonce_ring[i].issued_us) <= NONCE_TTL_US &&
            crypto::constant_time_eq(nonce_ring[i].bytes.data(), out_nonce.data(), NONCE_BYTES)) {
            nonce_ring[i].valid = false;
            ok = true;
            break;
        }
    }
    taskEXIT_CRITICAL(&nonce_lock);
    return ok;
}

bool verify_hmac(const std::array<uint8_t, NONCE_BYTES>& nonce,
                 const uint8_t* payload, size_t payload_len,
                 const char* sig_hex) {
    if (!lan_key_set) return false;
    uint8_t expected[32];
    if (!crypto::from_hex(sig_hex, expected, 32)) return false;
    crypto::HmacSha256 hmac;
    hmac.init(lan_key.data(), lan_key.size());
    hmac.update(nonce.data(), nonce.size());
    hmac.update(payload, payload_len);
    uint8_t actual[32];
    hmac.final(actual);
    return crypto::constant_time_eq(expected, actual, 32);
}

void set_cors_headers(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers",
                       "X-Nonce, X-Signature, Content-Type, X-Raw-Size, X-Raw-Sha256, X-Prog-Name, X-Prog-Pin");
    httpd_resp_set_hdr(req, "Access-Control-Expose-Headers", "X-Log-Seq, X-Stream-Start, X-Stream-End, X-Next-Nonce");
}

bool auth_empty_request(httpd_req_t* req) {
    char nonce_hex[NONCE_BYTES * 2 + 4] = {};
    char sig_hex[68] = {};
    if (httpd_req_get_hdr_value_str(req, "X-Nonce", nonce_hex, sizeof nonce_hex) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "X-Signature", sig_hex, sizeof sig_hex) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Missing X-Nonce or X-Signature");
        return false;
    }
    std::array<uint8_t, NONCE_BYTES> nonce{};
    if (!consume_nonce(nonce_hex, nonce) || !verify_hmac(nonce, nullptr, 0, sig_hex)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "HMAC-SHA256 verification failed");
        return false;
    }
    return true;
}


// Issues a fresh one-time challenge nonce and attaches it as X-Next-Nonce so steady-state
// telemetry polling takes 1 HTTP round-trip instead of 2 (instant 2x latency reduction!).
void attach_next_nonce(httpd_req_t* req, char out_hex[NONCE_BYTES * 2 + 1]) {
    std::array<uint8_t, NONCE_BYTES> n{};
    issue_nonce(n);
    crypto::to_hex(n.data(), n.size(), out_hex);
    httpd_resp_set_hdr(req, "X-Next-Nonce", out_hex);
}

// 30-line zero-bloat UDP mDNS responder for `balax.local` (replaces ~25 KB espressif/mdns component!).
void mdns_responder_task(void*) {
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) vTaskDelete(nullptr);
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse);
    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(5353);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof bind_addr);
    ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.251");
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof mreq);

    uint8_t pkt[39] = {
        0x00,0x00, 0x84,0x00, 0x00,0x00, 0x00,0x01, 0x00,0x00, 0x00,0x00,
        5,'b','a','l','a','x', 5,'l','o','c','a','l', 0,
        0x00,0x01, 0x80,0x01, 0x00,0x00, 0x00,120, 0x00,0x04, 0,0,0,0,
    };
    sockaddr_in mcast{};
    mcast.sin_family = AF_INET;
    mcast.sin_port = htons(5353);
    mcast.sin_addr.s_addr = inet_addr("224.0.0.251");

    uint8_t rx[256];
    for (;;) {
        const int r = recv(sock, rx, sizeof rx, 0);
        if (r > 18 && (rx[2] & 0x80) == 0) {
            const uint32_t ip = wifi::ip();
            if (ip != 0 && memmem(rx + 12, static_cast<size_t>(r - 12), "" "balax" "" "local", 12) != nullptr) {
                std::memcpy(pkt + 35, &ip, 4);
                sendto(sock, pkt, sizeof pkt, 0, reinterpret_cast<sockaddr*>(&mcast), sizeof mcast);
            }
        }
    }
}

esp_err_t handle_options(httpd_req_t* req) {
    set_cors_headers(req);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t handle_challenge(httpd_req_t* req) {
    set_cors_headers(req);
    std::array<uint8_t, NONCE_BYTES> n{};
    issue_nonce(n);

    char nonce_hex[NONCE_BYTES * 2 + 1];
    crypto::to_hex(n.data(), n.size(), nonce_hex);

    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_app_desc_t* app = esp_app_get_description();
    char body[256];
    const int len = snprintf(
        body, sizeof body,
        "{\"nonce\":\"%s\",\"partition\":\"%s\",\"version\":\"%s\","
        "\"compressed_ota\":true,\"sector_skip\":true,\"prog_vm\":true,\"has_factory\":%s,"
        "\"flash_erases\":%lu}",
        nonce_hex,
        running ? running->label : "unknown",
        app ? app->version : "unknown",
        g_has_factory ? "true" : "false",
        static_cast<unsigned long>(g_flash_erases_total));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, len);
}

// 4 KB Sector-Skip Differential Flash Writer:
// Reads existing sector first and skips erase+write whenever contents are byte-identical!
struct SectorSkipWriter {
    const esp_partition_t* part = nullptr;
    uint8_t* pending = nullptr;   // 4096 B
    uint8_t* existing = nullptr;  // 4096 B
    size_t pending_len = 0;
    size_t written_offset = 0;
    uint32_t sectors_erased = 0;
    uint32_t sectors_skipped = 0;
    crypto::Sha256 raw_sha;

    bool init(const esp_partition_t* p) {
        part = p;
        pending = static_cast<uint8_t*>(malloc(SECTOR_BYTES));
        existing = static_cast<uint8_t*>(malloc(SECTOR_BYTES));
        if (!pending || !existing) {
            cleanup();
            return false;
        }
        pending_len = 0;
        written_offset = 0;
        sectors_erased = 0;
        sectors_skipped = 0;
        raw_sha.init();
        return true;
    }

    void cleanup() {
        free(pending);
        free(existing);
        pending = nullptr;
        existing = nullptr;
    }

    esp_err_t flush_sector() {
        if (pending_len == 0) return ESP_OK;
        if (pending_len < SECTOR_BYTES) {
            std::memset(pending + pending_len, 0xFF, SECTOR_BYTES - pending_len);
        }
        if (written_offset + SECTOR_BYTES > part->size) {
            return ESP_ERR_OTA_VALIDATE_FAILED;
        }
        esp_err_t err = esp_partition_read(part, written_offset, existing, SECTOR_BYTES);
        if (err == ESP_OK && std::memcmp(existing, pending, SECTOR_BYTES) == 0) {
            ++sectors_skipped;
        } else {
            err = esp_partition_erase_range(part, written_offset, SECTOR_BYTES);
            if (err != ESP_OK) return err;
            err = esp_partition_write(part, written_offset, pending, SECTOR_BYTES);
            if (err != ESP_OK) return err;
            ++sectors_erased;
        }
        written_offset += SECTOR_BYTES;
        pending_len = 0;
        return ESP_OK;
    }

    esp_err_t write(const uint8_t* data, size_t len) {
        raw_sha.update(data, len);
        size_t pos = 0;
        while (pos < len) {
            const size_t take = std::min(SECTOR_BYTES - pending_len, len - pos);
            std::memcpy(pending + pending_len, data + pos, take);
            pending_len += take;
            pos += take;
            if (pending_len == SECTOR_BYTES) {
                const esp_err_t err = flush_sector();
                if (err != ESP_OK) return err;
            }
        }
        return ESP_OK;
    }
};

void reboot_task(void*) {
    vTaskDelay(pdMS_TO_TICKS(350));
    esp_restart();
}

esp_err_t handle_lan_ota(httpd_req_t* req) {
    set_cors_headers(req);
    char nonce_hex[NONCE_BYTES * 2 + 4] = {};
    char sig_hex[68] = {};
    if (httpd_req_get_hdr_value_str(req, "X-Nonce", nonce_hex, sizeof nonce_hex) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "X-Signature", sig_hex, sizeof sig_hex) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Missing X-Nonce or X-Signature");
        return ESP_FAIL;
    }
    std::array<uint8_t, NONCE_BYTES> nonce{};
    if (!consume_nonce(nonce_hex, nonce)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Invalid or expired challenge nonce");
        return ESP_FAIL;
    }
    uint8_t expected_sig[32];
    if (!lan_key_set || !crypto::from_hex(sig_hex, expected_sig, 32)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Malformed X-Signature");
        return ESP_FAIL;
    }

    char content_type[64] = {};
    httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof content_type);
    const bool compressed = (std::strstr(content_type, "application/x-balax-zlib") != nullptr);

    char raw_sha_hdr[68] = {};
    uint8_t expected_raw_sha[32] = {};
    const bool check_raw_sha =
        (httpd_req_get_hdr_value_str(req, "X-Raw-Sha256", raw_sha_hdr, sizeof raw_sha_hdr) == ESP_OK) &&
        crypto::from_hex(raw_sha_hdr, expected_raw_sha, 32);

    const esp_partition_t* update_part = esp_ota_get_next_update_partition(nullptr);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No target OTA partition");
        return ESP_FAIL;
    }

    // Stop any running mini-program & clear unpinned DRAM cache to maximize heap headroom for tinfl!
    prog_vm::stop();
    vTaskDelay(pdMS_TO_TICKS(30));

    SectorSkipWriter writer;
    if (!writer.init(update_part)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory for sector writer");
        return ESP_FAIL;
    }

    tinfl_decompressor* inflator = nullptr;
    uint8_t* dict = nullptr;
    size_t dict_ofs = 0;
    if (compressed) {
        inflator = static_cast<tinfl_decompressor*>(malloc(sizeof(tinfl_decompressor)));
        dict = static_cast<uint8_t*>(malloc(TINFL_LZ_DICT_SIZE));
        if (!inflator || !dict) {
            free(inflator);
            free(dict);
            writer.cleanup();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory for zlib decompressor");
            return ESP_FAIL;
        }
        tinfl_init(inflator);
    }

    auto cleanup_all = [&]() {
        free(inflator);
        free(dict);
        writer.cleanup();
        status_lcd::set_busy(false);
    };

    status_lcd::set_busy(true);
    status_lcd::draw_ota_progress(compressed ? "ZLIB + SKIP" : "RAW + SKIP", 0, update_part->label);

    crypto::HmacSha256 hmac;
    hmac.init(lan_key.data(), lan_key.size());
    hmac.update(nonce.data(), nonce.size());

    size_t wire_total = 0;
    size_t raw_total = 0;
    const size_t expected_wire = req->content_len;
    int last_pct = 0;
    bool stream_done = false;
    uint8_t rx_buf[1024];

    while (wire_total < expected_wire) {
        const size_t want = std::min(sizeof rx_buf, expected_wire - wire_total);
        const int r = httpd_req_recv(req, reinterpret_cast<char*>(rx_buf), want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            cleanup_all();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Connection dropped during OTA");
            return ESP_FAIL;
        }
        wire_total += static_cast<size_t>(r);
        hmac.update(rx_buf, static_cast<size_t>(r));

        if (!compressed) {
            const esp_err_t werr = writer.write(rx_buf, static_cast<size_t>(r));
            if (werr != ESP_OK) {
                cleanup_all();
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write error");
                return ESP_FAIL;
            }
            raw_total += static_cast<size_t>(r);
        } else {
            size_t in_pos = 0;
            const bool has_more = (wire_total < expected_wire);
            while (in_pos < static_cast<size_t>(r) && !stream_done) {
                size_t in_bytes = static_cast<size_t>(r) - in_pos;
                size_t out_bytes = TINFL_LZ_DICT_SIZE - dict_ofs;
                const uint32_t flags = TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_COMPUTE_ADLER32 |
                                       (has_more ? static_cast<uint32_t>(TINFL_FLAG_HAS_MORE_INPUT) : 0u);
                const tinfl_status st = tinfl_decompress(
                    inflator, rx_buf + in_pos, &in_bytes, dict, dict + dict_ofs, &out_bytes, flags);
                in_pos += in_bytes;
                if (out_bytes > 0) {
                    const esp_err_t werr = writer.write(dict + dict_ofs, out_bytes);
                    if (werr != ESP_OK) {
                        cleanup_all();
                        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write error");
                        return ESP_FAIL;
                    }
                    raw_total += out_bytes;
                    dict_ofs = (dict_ofs + out_bytes) & (TINFL_LZ_DICT_SIZE - 1);
                }
                if (st == TINFL_STATUS_DONE) {
                    stream_done = true;
                    break;
                }
                if (st < 0) {
                    cleanup_all();
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Corrupt zlib OTA stream");
                    return ESP_FAIL;
                }
                if (st == TINFL_STATUS_NEEDS_MORE_INPUT && in_bytes == 0 && out_bytes == 0) {
                    break;
                }
            }
        }

        const int pct = expected_wire > 0 ? static_cast<int>((wire_total * 100ULL) / expected_wire) : 0;
        if (pct >= last_pct + 5) {
            last_pct = pct;
            char detail[24];
            snprintf(detail, sizeof detail, "E:%lu S:%lu",
                     static_cast<unsigned long>(writer.sectors_erased),
                     static_cast<unsigned long>(writer.sectors_skipped));
            status_lcd::draw_ota_progress(compressed ? "ZLIB + SKIP" : "RAW + SKIP", pct, detail);
        }
    }

    if (compressed && !stream_done) {
        cleanup_all();
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete zlib stream");
        return ESP_FAIL;
    }

    if (writer.flush_sector() != ESP_OK) {
        cleanup_all();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash final flush failed");
        return ESP_FAIL;
    }

    // 1. Cryptographic HMAC-SHA256 verification over wire body
    uint8_t computed_sig[32];
    hmac.final(computed_sig);
    if (!crypto::constant_time_eq(expected_sig, computed_sig, 32)) {
        cleanup_all();
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "HMAC-SHA256 signature mismatch");
        return ESP_FAIL;
    }

    // 2. Optional decompressed raw SHA-256 verification
    if (check_raw_sha) {
        uint8_t actual_raw_sha[32];
        writer.raw_sha.final(actual_raw_sha);
        if (!crypto::constant_time_eq(expected_raw_sha, actual_raw_sha, 32)) {
            cleanup_all();
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Decompressed SHA-256 mismatch");
            return ESP_FAIL;
        }
    }

    // 3. ESP-IDF native image header/checksum/SHA-256 validation on flash before switching boot partition!
    esp_partition_pos_t part_pos = {.offset = update_part->address, .size = update_part->size};
    esp_image_metadata_t meta{};
    if (esp_image_verify(ESP_IMAGE_VERIFY, &part_pos, &meta) != ESP_OK) {
        cleanup_all();
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid ESP32 app image");
        return ESP_FAIL;
    }

    const uint32_t erased = writer.sectors_erased;
    const uint32_t skipped = writer.sectors_skipped;
    save_flash_wear_nvs(erased, skipped);

    if (esp_ota_set_boot_partition(update_part) != ESP_OK) {
        cleanup_all();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_set_boot_partition failed");
        return ESP_FAIL;
    }

    status_lcd::draw_ota_progress("REBOOTING!", 100, "VERIFIED OK");
    free(inflator);
    free(dict);
    writer.cleanup();

    char resp[256];
    const int resp_len = snprintf(
        resp, sizeof resp,
        "{\"ok\":true,\"partition\":\"%s\",\"compressed\":%s,\"bytes\":%u,\"raw_bytes\":%u,"
        "\"sectors_erased\":%lu,\"sectors_skipped\":%lu,\"flash_erases_total\":%lu}",
        update_part->label,
        compressed ? "true" : "false",
        static_cast<unsigned>(wire_total),
        static_cast<unsigned>(raw_total),
        static_cast<unsigned long>(erased),
        static_cast<unsigned long>(skipped),
        static_cast<unsigned long>(g_flash_erases_total));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, resp_len);
    xTaskCreate(reboot_task, "ota_reboot", 2048, nullptr, 5, nullptr);
    return ESP_OK;
}

esp_err_t handle_ota_enter(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;
    const esp_partition_t* factory =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr);
    if (!factory) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No factory OTA stub partition on this layout");
        return ESP_FAIL;
    }
    if (esp_ota_set_boot_partition(factory) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to select factory stub");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"entering\":\"factory\"}");
    xTaskCreate(reboot_task, "ota_enter", 2048, nullptr, 5, nullptr);
    return ESP_OK;
}

esp_err_t handle_lan_cmd(httpd_req_t* req) {
    set_cors_headers(req);
    char nonce_hex[NONCE_BYTES * 2 + 4] = {};
    char sig_hex[68] = {};
    if (httpd_req_get_hdr_value_str(req, "X-Nonce", nonce_hex, sizeof nonce_hex) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "X-Signature", sig_hex, sizeof sig_hex) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Missing X-Nonce or X-Signature");
        return ESP_FAIL;
    }
    std::array<uint8_t, NONCE_BYTES> nonce{};
    if (!consume_nonce(nonce_hex, nonce)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Invalid or expired challenge nonce");
        return ESP_FAIL;
    }
    char cmd[256] = {};
    const int len = httpd_req_recv(req, cmd, sizeof cmd - 1);
    if (len < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read command");
        return ESP_FAIL;
    }
    cmd[len] = '\0';
    if (!verify_hmac(nonce, reinterpret_cast<const uint8_t*>(cmd), static_cast<size_t>(len), sig_hex)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "HMAC-SHA256 signature mismatch");
        return ESP_FAIL;
    }

    uint32_t before_seq = 0;
    taskENTER_CRITICAL(&log_lock);
    before_seq = log_write_seq;
    taskEXIT_CRITICAL(&log_lock);

    FILE* old_stdout = stdout;
    FILE* mem_file = fopencookie(
        nullptr, "w",
        cookie_io_functions_t{
            .read = nullptr,
            .write = [](void*, const char* data, size_t size) -> ssize_t {
                append_to_log_ring(data, size);
                return static_cast<ssize_t>(size);
            },
            .seek = nullptr,
            .close = nullptr,
        });
    if (mem_file) {
        setvbuf(mem_file, nullptr, _IONBF, 0);
        stdout = mem_file;
    }

    int cmd_ret = 0;
    const esp_err_t err = esp_console_run(cmd, &cmd_ret);
    fflush(stdout);
    if (mem_file) {
        stdout = old_stdout;
        fclose(mem_file);
    }

    char* out_buf = reinterpret_cast<char*>(shared_http_buf);
    taskENTER_CRITICAL(&log_lock);
    const uint32_t after_seq = log_write_seq;
    const uint32_t avail = std::min<uint32_t>(after_seq - before_seq, SHARED_HTTP_BYTES - 1);
    copy_out_of_ring(after_seq - avail, out_buf, avail);
    taskEXIT_CRITICAL(&log_lock);
    size_t out_pos = avail;
    out_buf[out_pos] = '\0';

    if (err == ESP_ERR_NOT_FOUND) {
        out_pos = static_cast<size_t>(snprintf(out_buf, SHARED_HTTP_BYTES, "Unrecognized command: %s\n", cmd));
    }
    char next_nonce_hex[NONCE_BYTES * 2 + 1];
    attach_next_nonce(req, next_nonce_hex);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, out_buf, out_pos);
}

esp_err_t handle_lan_stream(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;

    uint32_t ack_seq = 0;
    char query[64] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof query) == ESP_OK) {
        char val[32] = {};
        if (httpd_query_key_value(query, "ack", val, sizeof val) == ESP_OK) {
            ack_seq = strtoul(val, nullptr, 10);
        }
    }

    uint32_t start_seq = 0;
    uint32_t end_seq = 0;
    size_t len = 0;
    if (stream_pull_cb) {
        len = stream_pull_cb(ack_seq, shared_http_buf, SHARED_HTTP_BYTES, start_seq, end_seq);
    }

    char start_hdr[16], end_hdr[16];
    snprintf(start_hdr, sizeof start_hdr, "%lu", static_cast<unsigned long>(start_seq));
    snprintf(end_hdr, sizeof end_hdr, "%lu", static_cast<unsigned long>(end_seq));
    char next_nonce_hex[NONCE_BYTES * 2 + 1];
    attach_next_nonce(req, next_nonce_hex);
    httpd_resp_set_hdr(req, "X-Stream-Start", start_hdr);
    httpd_resp_set_hdr(req, "X-Stream-End", end_hdr);
    httpd_resp_set_type(req, "application/octet-stream");
    return httpd_resp_send(req, reinterpret_cast<const char*>(shared_http_buf), len);
}

esp_err_t handle_lan_logs(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;

    uint32_t since = 0;
    char query[64] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof query) == ESP_OK) {
        char val[32] = {};
        if (httpd_query_key_value(query, "since", val, sizeof val) == ESP_OK) {
            since = strtoul(val, nullptr, 10);
        }
    }

    char chunk[1024];
    size_t copy_len = 0;
    taskENTER_CRITICAL(&log_lock);
    const uint32_t cur_seq = log_write_seq;
    const uint32_t earliest = (cur_seq > LOG_RING_BYTES) ? (cur_seq - LOG_RING_BYTES) : 0;
    if (since < earliest) since = earliest;
    if (since < cur_seq) {
        copy_len = std::min<size_t>(cur_seq - since, sizeof chunk);
        copy_out_of_ring(since, chunk, copy_len);
    }
    taskEXIT_CRITICAL(&log_lock);

    char seq_hdr[32];
    snprintf(seq_hdr, sizeof seq_hdr, "%lu", static_cast<unsigned long>(since + copy_len));
    httpd_resp_set_hdr(req, "X-Log-Seq", seq_hdr);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, chunk, copy_len);
}

// --- Mini-Program VM & Tier-0 DRAM Cache Endpoints ---

esp_err_t handle_prog_run(httpd_req_t* req) {
    set_cors_headers(req);
    char nonce_hex[NONCE_BYTES * 2 + 4] = {};
    char sig_hex[68] = {};
    if (httpd_req_get_hdr_value_str(req, "X-Nonce", nonce_hex, sizeof nonce_hex) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "X-Signature", sig_hex, sizeof sig_hex) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Missing X-Nonce or X-Signature");
        return ESP_FAIL;
    }
    std::array<uint8_t, NONCE_BYTES> nonce{};
    if (!consume_nonce(nonce_hex, nonce)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Invalid or expired challenge nonce");
        return ESP_FAIL;
    }

    // Mode A: zero-upload Tier-0 DRAM cache hit via ?sha256=<64-hex>
    char query[128] = {};
    char sha_q[70] = {};
    if (req->content_len == 0 &&
        httpd_req_get_url_query_str(req, query, sizeof query) == ESP_OK &&
        httpd_query_key_value(query, "sha256", sha_q, sizeof sha_q) == ESP_OK) {
        if (!verify_hmac(nonce, nullptr, 0, sig_hex)) {
            httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "HMAC-SHA256 mismatch");
            return ESP_FAIL;
        }
        const auto lr = prog_vm::start_from_cache(sha_q);
        char resp[256];
        const int n = snprintf(resp, sizeof resp,
                               "{\"ok\":%s,\"cache_hit\":%s,\"sha256\":\"%s\",\"error\":\"%s\"}",
                               lr.ok ? "true" : "false", lr.cache_hit ? "true" : "false",
                               lr.sha256, lr.error ? lr.error : "");
        httpd_resp_set_type(req, "application/json");
        if (!lr.ok && lr.error && std::strcmp(lr.error, "cache_miss") == 0) {
            httpd_resp_set_status(req, "404 Not Found");
        }
        return httpd_resp_send(req, resp, n);
    }

    // Mode B: full .bxp upload (authenticated via HMAC-SHA256(key, nonce || body))
    if (req->content_len < bxvm::HEADER_BYTES || req->content_len > bxvm::MAX_PROG_BYTES) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Program size out of bounds");
        return ESP_FAIL;
    }
    std::vector<uint8_t> body(req->content_len);
    size_t got = 0;
    while (got < body.size()) {
        const int r = httpd_req_recv(req, reinterpret_cast<char*>(body.data() + got), body.size() - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload interrupted");
            return ESP_FAIL;
        }
        got += static_cast<size_t>(r);
    }
    if (!verify_hmac(nonce, body.data(), body.size(), sig_hex)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "HMAC-SHA256 signature mismatch");
        return ESP_FAIL;
    }

    char prog_name[48] = "applet";
    httpd_req_get_hdr_value_str(req, "X-Prog-Name", prog_name, sizeof prog_name);
    char pin_hdr[8] = {};
    const bool pin = (httpd_req_get_hdr_value_str(req, "X-Prog-Pin", pin_hdr, sizeof pin_hdr) == ESP_OK &&
                      (pin_hdr[0] == '1' || pin_hdr[0] == 't'));

    const auto lr = prog_vm::start_from_bytes(prog_name, body.data(), body.size(), pin);
    char resp[256];
    const int n = snprintf(resp, sizeof resp,
                           "{\"ok\":%s,\"cache_hit\":%s,\"sha256\":\"%s\",\"bytes\":%u,\"error\":\"%s\"}",
                           lr.ok ? "true" : "false", lr.cache_hit ? "true" : "false",
                           lr.sha256, static_cast<unsigned>(body.size()), lr.error ? lr.error : "");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, n);
}

esp_err_t handle_prog_stop(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;
    prog_vm::stop();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t handle_prog_status(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;
    char* out = reinterpret_cast<char*>(shared_http_buf);
    const size_t n = prog_vm::format_status_json(out, SHARED_HTTP_BYTES);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, n);
}

esp_err_t handle_prog_cache_get(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;
    char* out = reinterpret_cast<char*>(shared_http_buf);
    const size_t n = prog_vm::format_cache_json(out, SHARED_HTTP_BYTES);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, n);
}

esp_err_t handle_prog_cache_pin(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;
    char query[128] = {};
    char sha_q[70] = {};
    char pin_q[8] = "1";
    if (httpd_req_get_url_query_str(req, query, sizeof query) != ESP_OK ||
        httpd_query_key_value(query, "sha256", sha_q, sizeof sha_q) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing sha256");
        return ESP_FAIL;
    }
    httpd_query_key_value(query, "pin", pin_q, sizeof pin_q);
    const bool ok = prog_vm::set_pinned(sha_q, pin_q[0] == '1' || pin_q[0] == 't');
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

esp_err_t handle_prog_cache_delete(httpd_req_t* req) {
    set_cors_headers(req);
    if (!auth_empty_request(req)) return ESP_FAIL;
    prog_vm::clear_cache();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

}  // namespace

void set_stream_callback(StreamPullCallback cb) { stream_pull_cb = cb; }

OtaStats ota_stats() {
    return OtaStats{g_flash_erases_total, g_last_sectors_erased, g_last_sectors_skipped, g_has_factory};
}

esp_err_t start(std::span<const uint8_t, 32> auth_key) {
    std::copy(auth_key.begin(), auth_key.end(), lan_key.begin());
    lan_key_set = true;
    load_flash_wear_nvs();
    prog_vm::init();

    if (!prev_vprintf) {
        prev_vprintf = esp_log_set_vprintf(tee_vprintf);
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 6144;
    cfg.max_uri_handlers = 16;
    cfg.recv_wait_timeout = 15;
    cfg.send_wait_timeout = 15;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    // Pinned with Wi-Fi rather than left to float: commands print floats, and FreeRTOS pins a
    // task to whichever core it first touches the FPU on, which could strand OTA inflating and
    // flash writes next to the balance loop on core 1.
    cfg.core_id = HTTPD_CORE;

    httpd_handle_t server = nullptr;
    const esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    static constexpr httpd_uri_t ROUTES[] = {
        {"/api/challenge",      HTTP_GET,     handle_challenge,         nullptr},
        {"/api/ota",            HTTP_POST,    handle_lan_ota,           nullptr},
        {"/api/ota/enter",      HTTP_POST,    handle_ota_enter,         nullptr},
        {"/api/cmd",            HTTP_POST,    handle_lan_cmd,           nullptr},
        {"/api/logs",           HTTP_GET,     handle_lan_logs,          nullptr},
        {"/api/stream",         HTTP_GET,     handle_lan_stream,        nullptr},
        {"/api/prog/run",       HTTP_POST,    handle_prog_run,          nullptr},
        {"/api/prog/stop",      HTTP_POST,    handle_prog_stop,         nullptr},
        {"/api/prog/status",    HTTP_GET,     handle_prog_status,       nullptr},
        {"/api/prog/cache",     HTTP_GET,     handle_prog_cache_get,    nullptr},
        {"/api/prog/cache/pin", HTTP_POST,    handle_prog_cache_pin,    nullptr},
        {"/api/prog/cache",     HTTP_DELETE,  handle_prog_cache_delete, nullptr},
        {"/api/*",              HTTP_OPTIONS, handle_options,           nullptr},
    };
    for (const httpd_uri_t& route : ROUTES) {
        httpd_register_uri_handler(server, &route);
    }
    // Integer-only, so free to run on either core: `balax.local` keeps answering while an OTA
    // update or a mini-program keeps core 0 busy (the balance loop on core 1 outranks it).
    xTaskCreatePinnedToCore(mdns_responder_task, "mdns_mini", 2560, nullptr, MDNS_PRIORITY, nullptr, tskNO_AFFINITY);
    ESP_LOGI(TAG, "Slim LAN server started (mDNS + 1-RTT nonce + zlib/sector-skip OTA + BXVM)");
    return ESP_OK;
}

}  // namespace lan_server
