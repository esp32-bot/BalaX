#pragma once

// Zero-drop compressed telemetry streaming queue (`BXZ1` binary delta+varint format):
//
// - Only records when a streaming client (`balax-remote` or an agent) is actively subscribed.
// - Compresses 100 Hz `robot::TraceSample` blocks into compact frames in RAM (~3-6x smaller
//   than raw structs, ~15x smaller than CSV/JSON).
// - As soon as the client acknowledges (`ack_tick`), transmitted RAM chunks and flash spillover
//   blocks are immediately freed (`free()`).
// - If the LAN connection drops while streaming and the RAM queue hits its byte cap, compressed
//   chunks spill over into flash (`0x3e0000`, 64 KB) and are drained & freed once the client
//   reconnects and ACKs them.

#include <cstddef>
#include <cstdint>

#include "trace_sample.h"

namespace telemetry_stream {

struct Stats {
    bool active = false;
    uint32_t last_ack_tick = 0;
    uint32_t latest_tick = 0;
    uint32_t ram_chunks = 0;
    uint32_t ram_bytes = 0;
    uint32_t flash_chunks = 0;
    uint32_t flash_bytes = 0;
    uint32_t total_samples_sent = 0;
    uint32_t total_bytes_compressed = 0;
};

// Called from the 100 Hz control loop (or publisher): fast and non-blocking.
void push_sample(const robot::TraceSample& sample);

// Activates streaming (or keeps lease alive), frees all chunks with end_tick <= ack_tick,
// flushes any pending samples into a compressed `BXZ1` block, and copies up to `max_out`
// bytes of compressed `BXZ1` blocks into `out`.
// Returns number of bytes written to `out`, and sets `out_start_tick` / `out_end_tick`.
size_t pull_compressed(uint32_t ack_tick, uint8_t* out, size_t max_out,
                       uint32_t& out_start_tick, uint32_t& out_end_tick);

// Stops streaming and frees all RAM and flash buffers immediately.
void stop_and_free();

Stats stats();

// Pure C++ BXZ1 block compressor / decompressor (exposed for host unit tests).
// Header: "BXZ1" (4B) | start_tick (u32 LE) | end_tick (u32 LE) | count (u16 LE) | payload_len (u16 LE)
size_t encode_bxz1_block(const robot::TraceSample* samples, size_t count, uint8_t* out, size_t max_out);
size_t decode_bxz1_block(const uint8_t* data, size_t len, robot::TraceSample* out, size_t max_samples);

}  // namespace telemetry_stream
