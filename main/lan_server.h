#pragma once

// Slim authenticated LAN HTTP server (zero mbedTLS / x509 / esp_https_ota dependency).
// Handles:
//   GET  /api/challenge
//   POST /api/cmd
//   GET  /api/logs
//   GET  /api/stream
//   POST /api/ota        (uncompressed OR zlib-compressed via Mask ROM tinfl + 4 KB sector-skip!)
//   POST /api/ota/enter  (boots into factory ota_stub if present)
//   POST /api/prog/run   (raw .bxp upload OR ?sha256=<hex> zero-transfer DRAM cache hit)
//   POST /api/prog/stop
//   GET  /api/prog/status
//   GET  /api/prog/cache
//   POST /api/prog/cache/pin
//   DELETE /api/prog/cache

#include <cstddef>
#include <cstdint>
#include <span>

#include "esp_err.h"

namespace lan_server {

using StreamPullCallback = size_t (*)(uint32_t ack_seq, uint8_t* out, size_t max_len,
                                      uint32_t& out_start_seq, uint32_t& out_end_seq);

struct OtaStats {
    uint32_t flash_erases_total = 0;
    uint32_t last_sectors_erased = 0;
    uint32_t last_sectors_skipped = 0;
    bool has_factory = false;
};

void set_stream_callback(StreamPullCallback cb);
esp_err_t start(std::span<const uint8_t, 32> auth_key);
OtaStats ota_stats();

}  // namespace lan_server
