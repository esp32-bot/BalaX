#include "telemetry_stream.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#ifdef ESP_PLATFORM
#include "esp_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#endif

namespace telemetry_stream {

namespace {

constexpr size_t BXZ1_HEADER_BYTES = 16;
constexpr size_t BLOCK_SAMPLES = 50;          // 0.5s per compressed block
constexpr size_t MAX_RAM_CHUNKS = 48;         // up to 48 compressed chunks in RAM (~24s at 100 Hz)
constexpr size_t MAX_RAM_BYTES = 24 * 1024;   // 24 KB maximum RAM usage before spilling to flash
constexpr int64_t STREAM_LEASE_US = 60'000'000; // 60s timeout after client vanishes completely
#ifdef ESP_PLATFORM
constexpr uint32_t FLASH_BASE_ADDR = 0x3e0000;
constexpr uint32_t FLASH_SIZE_BYTES = 0x10000; // 64 KB spillover flash window (~120s of compressed 100Hz data)
constexpr uint32_t FLASH_SECTOR_BYTES = 4096;
#endif

struct QuantizedSample {
    uint32_t tick;
    uint8_t state_fault;  // (state & 0x0F) | ((fault & 0x0F) << 4)
    int8_t motor_l;
    int8_t motor_r;
    int32_t pitch_cdeg;   // pitch_deg * 100
    int32_t rate_ddps;    // pitch_dps * 10
    int32_t angle_i_d;    // angle_i * 10
    int32_t rate_i_d;     // rate_i * 10
    int32_t speed_c;      // speed * 100
    int32_t distance_d;   // distance * 10
    int32_t power_d;      // power * 10
    int32_t accel_x_cg;   // accel_x_g * 100
    int32_t accel_y_cg;   // accel_y_g * 100
    int32_t accel_z_cg;   // accel_z_g * 100
    int32_t gyro_x_ddps;  // gyro_x_dps * 10
    int32_t gyro_y_ddps;  // gyro_y_dps * 10
    int32_t gyro_z_ddps;  // gyro_z_dps * 10
    int32_t yaw_ddeg;     // yaw_deg * 10
};

QuantizedSample quantize(const robot::TraceSample& s) {
    return {
        .tick = s.tick,
        .state_fault = static_cast<uint8_t>((static_cast<uint8_t>(s.state) & 0x0F) |
                                            ((static_cast<uint8_t>(s.fault) & 0x0F) << 4)),
        .motor_l = s.motor_l,
        .motor_r = s.motor_r,
        .pitch_cdeg = static_cast<int32_t>(std::lround(s.pitch_deg * 100.0f)),
        .rate_ddps = static_cast<int32_t>(std::lround(s.pitch_dps * 10.0f)),
        .angle_i_d = static_cast<int32_t>(std::lround(s.angle_i * 10.0f)),
        .rate_i_d = static_cast<int32_t>(std::lround(s.rate_i * 10.0f)),
        .speed_c = static_cast<int32_t>(std::lround(s.speed * 100.0f)),
        .distance_d = static_cast<int32_t>(std::lround(s.distance * 10.0f)),
        .power_d = static_cast<int32_t>(std::lround(s.power * 10.0f)),
        .accel_x_cg = static_cast<int32_t>(std::lround(s.accel_x_g * 100.0f)),
        .accel_y_cg = static_cast<int32_t>(std::lround(s.accel_y_g * 100.0f)),
        .accel_z_cg = static_cast<int32_t>(std::lround(s.accel_z_g * 100.0f)),
        .gyro_x_ddps = static_cast<int32_t>(std::lround(s.gyro_x_dps * 10.0f)),
        .gyro_y_ddps = static_cast<int32_t>(std::lround(s.gyro_y_dps * 10.0f)),
        .gyro_z_ddps = static_cast<int32_t>(std::lround(s.gyro_z_dps * 10.0f)),
        .yaw_ddeg = static_cast<int32_t>(std::lround(s.yaw_deg * 10.0f)),
    };
}

robot::TraceSample dequantize(const QuantizedSample& q) {
    return {
        .tick = q.tick,
        .state = static_cast<balance::State>(q.state_fault & 0x0F),
        .fault = static_cast<balance::Fault>((q.state_fault >> 4) & 0x0F),
        .motor_l = q.motor_l,
        .motor_r = q.motor_r,
        .pitch_deg = q.pitch_cdeg / 100.0f,
        .pitch_dps = q.rate_ddps / 10.0f,
        .angle_i = q.angle_i_d / 10.0f,
        .rate_i = q.rate_i_d / 10.0f,
        .speed = q.speed_c / 100.0f,
        .distance = q.distance_d / 10.0f,
        .power = q.power_d / 10.0f,
        .accel_x_g = q.accel_x_cg / 100.0f,
        .accel_y_g = q.accel_y_cg / 100.0f,
        .accel_z_g = q.accel_z_cg / 100.0f,
        .gyro_x_dps = q.gyro_x_ddps / 10.0f,
        .gyro_y_dps = q.gyro_y_ddps / 10.0f,
        .gyro_z_dps = q.gyro_z_ddps / 10.0f,
        .yaw_deg = q.yaw_ddeg / 10.0f,
    };
}

uint32_t zigzag_encode(int32_t v) {
    return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31);
}

int32_t zigzag_decode(uint32_t u) {
    return static_cast<int32_t>((u >> 1) ^ -static_cast<int32_t>(u & 1));
}

size_t write_varint(uint32_t val, uint8_t* out) {
    size_t n = 0;
    while (val >= 0x80) {
        out[n++] = static_cast<uint8_t>((val & 0x7F) | 0x80);
        val >>= 7;
    }
    out[n++] = static_cast<uint8_t>(val);
    return n;
}

size_t read_varint(const uint8_t* data, size_t max_len, uint32_t& out) {
    out = 0;
    uint32_t shift = 0;
    for (size_t i = 0; i < max_len && i < 5; ++i) {
        const uint8_t b = data[i];
        out |= static_cast<uint32_t>(b & 0x7F) << shift;
        if ((b & 0x80) == 0) {
            return i + 1;
        }
        shift += 7;
    }
    return 0;
}

void write_u16_le(uint16_t v, uint8_t* out) {
    out[0] = static_cast<uint8_t>(v & 0xFF);
    out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void write_u32_le(uint32_t v, uint8_t* out) {
    out[0] = static_cast<uint8_t>(v & 0xFF);
    out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    out[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint16_t read_u16_le(const uint8_t* in) {
    return static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
}

uint32_t read_u32_le(const uint8_t* in) {
    return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
           (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

struct Chunk {
    uint32_t start_tick = 0;
    uint32_t end_tick = 0;
    uint16_t byte_len = 0;
    bool in_flash = false;
    uint32_t flash_offset = 0;
    uint8_t* ram_data = nullptr;
};

#ifdef ESP_PLATFORM
portMUX_TYPE stream_lock = portMUX_INITIALIZER_UNLOCKED;
#define STREAM_ENTER_CRITICAL() taskENTER_CRITICAL(&stream_lock)
#define STREAM_EXIT_CRITICAL() taskEXIT_CRITICAL(&stream_lock)
#else
#define STREAM_ENTER_CRITICAL() ((void)0)
#define STREAM_EXIT_CRITICAL() ((void)0)
#endif

bool stream_active = false;
bool flushing = false;
int64_t last_poll_us = 0;
uint32_t last_ack_tick = 0;
uint32_t latest_tick = 0;
uint32_t total_samples_sent = 0;
uint32_t total_bytes_compressed = 0;

robot::TraceSample staging[BLOCK_SAMPLES];
size_t staging_count = 0;

// Keep scratch buffers in BSS rather than on the `robot` task stack; guarded by `flushing`.
robot::TraceSample flush_local_samples[BLOCK_SAMPLES];
uint8_t flush_encoded[2048];

Chunk chunks[MAX_RAM_CHUNKS];
size_t chunk_head = 0;
size_t chunk_count = 0;
size_t current_ram_bytes = 0;
uint32_t flash_write_offset = 0;
uint32_t flash_erased_until = 0;

int64_t now_us() {
#ifdef ESP_PLATFORM
    return esp_timer_get_time();
#else
    return 0;
#endif
}

bool write_to_flash_spillover(const uint8_t* data, size_t len, uint32_t& out_offset) {
#ifdef ESP_PLATFORM
    if (flash_write_offset + len > FLASH_SIZE_BYTES) {
        return false;
    }
    while (flash_erased_until < flash_write_offset + len) {
        if (esp_flash_erase_region(nullptr, FLASH_BASE_ADDR + flash_erased_until, FLASH_SECTOR_BYTES) != ESP_OK) {
            return false;
        }
        flash_erased_until += FLASH_SECTOR_BYTES;
    }
    if (esp_flash_write(nullptr, data, FLASH_BASE_ADDR + flash_write_offset, len) != ESP_OK) {
        return false;
    }
    out_offset = flash_write_offset;
    flash_write_offset += static_cast<uint32_t>((len + 3) & ~3u);
    return true;
#else
    (void)data;
    (void)len;
    (void)out_offset;
    return false;
#endif
}

bool read_from_flash_spillover(uint32_t offset, uint8_t* out, size_t len) {
#ifdef ESP_PLATFORM
    return esp_flash_read(nullptr, out, FLASH_BASE_ADDR + offset, len) == ESP_OK;
#else
    (void)offset;
    (void)out;
    (void)len;
    return false;
#endif
}

void flush_staging_unlocked() {
    if (staging_count == 0 || flushing) {
        return;
    }
    flushing = true;
    const size_t n = staging_count;
    memcpy(flush_local_samples, staging, n * sizeof(robot::TraceSample));
    staging_count = 0;
    const uint32_t block_start_tick = flush_local_samples[0].tick;
    const uint32_t block_end_tick = flush_local_samples[n - 1].tick;
    STREAM_EXIT_CRITICAL();

    const size_t enc_len = encode_bxz1_block(flush_local_samples, n, flush_encoded, sizeof flush_encoded);
    if (enc_len == 0) {
        STREAM_ENTER_CRITICAL();
        flushing = false;
        return;
    }

    uint8_t* ram_copy = nullptr;
    bool spill_to_flash = false;
    STREAM_ENTER_CRITICAL();
    if (!stream_active) {
        flushing = false;
        return;
    }
    if (current_ram_bytes + enc_len <= MAX_RAM_BYTES && chunk_count < MAX_RAM_CHUNKS) {
        STREAM_EXIT_CRITICAL();
        ram_copy = static_cast<uint8_t*>(malloc(enc_len));
        if (ram_copy) {
            memcpy(ram_copy, flush_encoded, enc_len);
        } else {
            spill_to_flash = true;
        }
        STREAM_ENTER_CRITICAL();
        if (!stream_active) {
            if (ram_copy) {
                STREAM_EXIT_CRITICAL();
                free(ram_copy);
                STREAM_ENTER_CRITICAL();
            }
            flushing = false;
            return;
        }
    } else {
        spill_to_flash = true;
    }

    uint32_t flash_off = 0;
    if (spill_to_flash) {
        STREAM_EXIT_CRITICAL();
        const bool ok = write_to_flash_spillover(flush_encoded, enc_len, flash_off);
        STREAM_ENTER_CRITICAL();
        if (!stream_active) {
            flushing = false;
            return;
        }
        if (!ok) {
            // If both RAM and Flash spillover are full, drop the oldest unacked chunk so newest survives
            if (chunk_count > 0) {
                Chunk& oldest = chunks[chunk_head];
                uint8_t* ptr = oldest.ram_data;
                if (ptr) {
                    current_ram_bytes -= oldest.byte_len;
                    oldest.ram_data = nullptr;
                }
                chunk_head = (chunk_head + 1) % MAX_RAM_CHUNKS;
                --chunk_count;
                if (ptr) {
                    STREAM_EXIT_CRITICAL();
                    free(ptr);
                    STREAM_ENTER_CRITICAL();
                }
            }
            flushing = false;
            return;
        }
    }

    if (chunk_count == MAX_RAM_CHUNKS) {
        Chunk& oldest = chunks[chunk_head];
        uint8_t* ptr = oldest.ram_data;
        if (ptr) {
            current_ram_bytes -= oldest.byte_len;
            oldest.ram_data = nullptr;
        }
        chunk_head = (chunk_head + 1) % MAX_RAM_CHUNKS;
        --chunk_count;
        if (ptr) {
            STREAM_EXIT_CRITICAL();
            free(ptr);
            STREAM_ENTER_CRITICAL();
            if (!stream_active) {
                if (ram_copy) {
                    STREAM_EXIT_CRITICAL();
                    free(ram_copy);
                    STREAM_ENTER_CRITICAL();
                }
                flushing = false;
                return;
            }
        }
    }

    const size_t slot = (chunk_head + chunk_count) % MAX_RAM_CHUNKS;
    chunks[slot] = {
        .start_tick = block_start_tick,
        .end_tick = block_end_tick,
        .byte_len = static_cast<uint16_t>(enc_len),
        .in_flash = spill_to_flash,
        .flash_offset = flash_off,
        .ram_data = ram_copy,
    };
    if (ram_copy) {
        current_ram_bytes += enc_len;
    }
    ++chunk_count;
    total_samples_sent += static_cast<uint32_t>(n);
    total_bytes_compressed += static_cast<uint32_t>(enc_len);
    flushing = false;
}

}  // namespace

size_t encode_bxz1_block(const robot::TraceSample* samples, size_t count, uint8_t* out, size_t max_out) {
    if (count == 0 || max_out < BXZ1_HEADER_BYTES + 48) {
        return 0;
    }
    size_t pos = BXZ1_HEADER_BYTES;
    QuantizedSample prev{};

    for (size_t i = 0; i < count; ++i) {
        if (pos + 90 > max_out) {
            count = i;
            break;
        }
        const QuantizedSample cur = quantize(samples[i]);
        const int32_t d_tick = (i == 0) ? 0 : static_cast<int32_t>(cur.tick - prev.tick - 1);
        const int32_t d_state = static_cast<int32_t>(cur.state_fault) - static_cast<int32_t>(prev.state_fault);
        const int32_t d_ml = static_cast<int32_t>(cur.motor_l) - static_cast<int32_t>(prev.motor_l);
        const int32_t d_mr = static_cast<int32_t>(cur.motor_r) - static_cast<int32_t>(prev.motor_r);
        const int32_t d_pitch = cur.pitch_cdeg - prev.pitch_cdeg;
        const int32_t d_rate = cur.rate_ddps - prev.rate_ddps;
        const int32_t d_ai = cur.angle_i_d - prev.angle_i_d;
        const int32_t d_ri = cur.rate_i_d - prev.rate_i_d;
        const int32_t d_spd = cur.speed_c - prev.speed_c;
        const int32_t d_dst = cur.distance_d - prev.distance_d;
        const int32_t d_pwr = cur.power_d - prev.power_d;
        const int32_t d_ax = cur.accel_x_cg - prev.accel_x_cg;
        const int32_t d_ay = cur.accel_y_cg - prev.accel_y_cg;
        const int32_t d_az = cur.accel_z_cg - prev.accel_z_cg;
        const int32_t d_gx = cur.gyro_x_ddps - prev.gyro_x_ddps;
        const int32_t d_gy = cur.gyro_y_ddps - prev.gyro_y_ddps;
        const int32_t d_gz = cur.gyro_z_ddps - prev.gyro_z_ddps;
        const int32_t d_yaw = cur.yaw_ddeg - prev.yaw_ddeg;

        pos += write_varint(zigzag_encode(d_tick), out + pos);
        pos += write_varint(zigzag_encode(d_state), out + pos);
        pos += write_varint(zigzag_encode(d_ml), out + pos);
        pos += write_varint(zigzag_encode(d_mr), out + pos);
        pos += write_varint(zigzag_encode(d_pitch), out + pos);
        pos += write_varint(zigzag_encode(d_rate), out + pos);
        pos += write_varint(zigzag_encode(d_ai), out + pos);
        pos += write_varint(zigzag_encode(d_ri), out + pos);
        pos += write_varint(zigzag_encode(d_spd), out + pos);
        pos += write_varint(zigzag_encode(d_dst), out + pos);
        pos += write_varint(zigzag_encode(d_pwr), out + pos);
        pos += write_varint(zigzag_encode(d_ax), out + pos);
        pos += write_varint(zigzag_encode(d_ay), out + pos);
        pos += write_varint(zigzag_encode(d_az), out + pos);
        pos += write_varint(zigzag_encode(d_gx), out + pos);
        pos += write_varint(zigzag_encode(d_gy), out + pos);
        pos += write_varint(zigzag_encode(d_gz), out + pos);
        pos += write_varint(zigzag_encode(d_yaw), out + pos);
        prev = cur;
    }

    const uint16_t payload_len = static_cast<uint16_t>(pos - BXZ1_HEADER_BYTES);
    out[0] = 'B';
    out[1] = 'X';
    out[2] = 'Z';
    out[3] = '2';
    write_u32_le(samples[0].tick, out + 4);
    write_u32_le(samples[count - 1].tick, out + 8);
    write_u16_le(static_cast<uint16_t>(count), out + 12);
    write_u16_le(payload_len, out + 14);
    return pos;
}

size_t decode_bxz1_block(const uint8_t* data, size_t len, robot::TraceSample* out, size_t max_samples) {
    if (len < BXZ1_HEADER_BYTES || data[0] != 'B' || data[1] != 'X' || data[2] != 'Z' ||
        (data[3] != '1' && data[3] != '2')) {
        return 0;
    }
    const int num_fields = (data[3] == '2') ? 18 : 11;
    const uint32_t start_tick = read_u32_le(data + 4);
    const uint16_t count = read_u16_le(data + 12);
    const uint16_t payload_len = read_u16_le(data + 14);
    if (BXZ1_HEADER_BYTES + payload_len > len) {
        return 0;
    }

    size_t pos = BXZ1_HEADER_BYTES;
    const size_t end = BXZ1_HEADER_BYTES + payload_len;
    QuantizedSample prev{};
    size_t decoded = 0;

    for (uint16_t i = 0; i < count && decoded < max_samples; ++i) {
        int32_t deltas[18] = {};
        for (int f = 0; f < num_fields; ++f) {
            uint32_t u = 0;
            const size_t n = read_varint(data + pos, end - pos, u);
            if (n == 0) {
                return decoded;
            }
            pos += n;
            deltas[f] = zigzag_decode(u);
        }
        QuantizedSample cur{};
        cur.tick = (i == 0) ? start_tick : static_cast<uint32_t>(static_cast<int32_t>(prev.tick) + 1 + deltas[0]);
        cur.state_fault = static_cast<uint8_t>(static_cast<int32_t>(prev.state_fault) + deltas[1]);
        cur.motor_l = static_cast<int8_t>(static_cast<int32_t>(prev.motor_l) + deltas[2]);
        cur.motor_r = static_cast<int8_t>(static_cast<int32_t>(prev.motor_r) + deltas[3]);
        cur.pitch_cdeg = prev.pitch_cdeg + deltas[4];
        cur.rate_ddps = prev.rate_ddps + deltas[5];
        cur.angle_i_d = prev.angle_i_d + deltas[6];
        cur.rate_i_d = prev.rate_i_d + deltas[7];
        cur.speed_c = prev.speed_c + deltas[8];
        cur.distance_d = prev.distance_d + deltas[9];
        cur.power_d = prev.power_d + deltas[10];
        cur.accel_x_cg = prev.accel_x_cg + deltas[11];
        cur.accel_y_cg = prev.accel_y_cg + deltas[12];
        cur.accel_z_cg = prev.accel_z_cg + deltas[13];
        cur.gyro_x_ddps = prev.gyro_x_ddps + deltas[14];
        cur.gyro_y_ddps = prev.gyro_y_ddps + deltas[15];
        cur.gyro_z_ddps = prev.gyro_z_ddps + deltas[16];
        cur.yaw_ddeg = prev.yaw_ddeg + deltas[17];
        out[decoded++] = dequantize(cur);
        prev = cur;
    }
    return decoded;
}

void push_sample(const robot::TraceSample& sample) {
    STREAM_ENTER_CRITICAL();
    latest_tick = sample.tick;
    if (!stream_active) {
        STREAM_EXIT_CRITICAL();
        return;
    }
    const int64_t t = now_us();
    if (t > 0 && last_poll_us > 0 && (t - last_poll_us) > STREAM_LEASE_US) {
        STREAM_EXIT_CRITICAL();
        stop_and_free();
        return;
    }
    if (staging_count < BLOCK_SAMPLES) {
        staging[staging_count++] = sample;
    }
    if (staging_count >= BLOCK_SAMPLES && !flushing) {
        flush_staging_unlocked();
    }
    STREAM_EXIT_CRITICAL();
}

size_t pull_compressed(uint32_t ack_tick, uint8_t* out, size_t max_out,
                       uint32_t& out_start_tick, uint32_t& out_end_tick) {
    STREAM_ENTER_CRITICAL();
    stream_active = true;
    last_poll_us = now_us();
    if (ack_tick > last_ack_tick) {
        last_ack_tick = ack_tick;
    }

    // Free all chunks already acknowledged by the receiver!
    bool any_flash_left = false;
    while (chunk_count > 0) {
        Chunk& front = chunks[chunk_head];
        if (front.end_tick <= last_ack_tick) {
            if (front.ram_data) {
                uint8_t* ptr = front.ram_data;
                front.ram_data = nullptr;
                current_ram_bytes -= front.byte_len;
                STREAM_EXIT_CRITICAL();
                free(ptr);
                STREAM_ENTER_CRITICAL();
            }
            chunk_head = (chunk_head + 1) % MAX_RAM_CHUNKS;
            --chunk_count;
        } else {
            break;
        }
    }
    for (size_t i = 0; i < chunk_count; ++i) {
        if (chunks[(chunk_head + i) % MAX_RAM_CHUNKS].in_flash) {
            any_flash_left = true;
        }
    }
    if (!any_flash_left) {
        flash_write_offset = 0;
        flash_erased_until = 0;
    }

    // Flush any staged samples into a compressed chunk right now so low-latency polling gets them immediately
    if (staging_count > 0 && !flushing) {
        flush_staging_unlocked();
    }

    size_t written = 0;
    out_start_tick = 0;
    out_end_tick = 0;
    for (size_t i = 0; i < chunk_count; ++i) {
        const Chunk& c = chunks[(chunk_head + i) % MAX_RAM_CHUNKS];
        if (written + c.byte_len > max_out) {
            break;
        }
        if (out_start_tick == 0) {
            out_start_tick = c.start_tick;
        }
        out_end_tick = c.end_tick;
        if (!c.in_flash && c.ram_data) {
            memcpy(out + written, c.ram_data, c.byte_len);
            written += c.byte_len;
        } else if (c.in_flash) {
            const uint32_t off = c.flash_offset;
            const uint16_t len = c.byte_len;
            STREAM_EXIT_CRITICAL();
            const bool ok = read_from_flash_spillover(off, out + written, len);
            STREAM_ENTER_CRITICAL();
            if (ok) {
                written += len;
            } else {
                break;
            }
        }
    }
    STREAM_EXIT_CRITICAL();
    return written;
}

void stop_and_free() {
    STREAM_ENTER_CRITICAL();
    stream_active = false;
    staging_count = 0;
    while (chunk_count > 0) {
        Chunk& front = chunks[chunk_head];
        uint8_t* ptr = front.ram_data;
        front.ram_data = nullptr;
        chunk_head = (chunk_head + 1) % MAX_RAM_CHUNKS;
        --chunk_count;
        if (ptr) {
            STREAM_EXIT_CRITICAL();
            free(ptr);
            STREAM_ENTER_CRITICAL();
        }
    }
    current_ram_bytes = 0;
    flash_write_offset = 0;
    flash_erased_until = 0;
    STREAM_EXIT_CRITICAL();
}

Stats stats() {
    STREAM_ENTER_CRITICAL();
    Stats s{};
    s.active = stream_active;
    s.last_ack_tick = last_ack_tick;
    s.latest_tick = latest_tick;
    s.total_samples_sent = total_samples_sent;
    s.total_bytes_compressed = total_bytes_compressed;
    for (size_t i = 0; i < chunk_count; ++i) {
        const Chunk& c = chunks[(chunk_head + i) % MAX_RAM_CHUNKS];
        if (c.in_flash) {
            ++s.flash_chunks;
            s.flash_bytes += c.byte_len;
        } else {
            ++s.ram_chunks;
            s.ram_bytes += c.byte_len;
        }
    }
    STREAM_EXIT_CRITICAL();
    return s;
}

}  // namespace telemetry_stream
