#pragma once

// Sandboxed Core 0 Mini-Program VM runtime + Tier-0 Zero-Flash-Wear DRAM Applet Cache.

#include <cstddef>
#include <cstdint>

#include "prog_vm_core.h"

namespace prog_vm {

struct RuntimeStats {
    bool running = false;
    bool last_ok = false;
    bool last_aborted = false;
    uint32_t last_steps = 0;
    uint32_t runs_total = 0;
    uint32_t cache_hits = 0;
    uint32_t cache_misses = 0;
    uint32_t cache_evictions = 0;
    size_t cache_used_bytes = 0;
    size_t cache_budget_bytes = bxvm::DRAM_CACHE_BUDGET_BYTES;
    size_t cache_entries = 0;
};

struct LaunchResult {
    bool ok = false;
    bool cache_hit = false;
    const char* error = nullptr;
    char sha256[65] = {};
};

// Initializes mutexes (safe to call once at boot).
void init();

// Validates, caches (in Tier-0 DRAM cache), and launches a .bxp program on Core 0.
LaunchResult start_from_bytes(const char* name, const uint8_t* data, size_t len, bool pin = false);

// Launches an already-cached .bxp program by its 64-char SHA-256 hex digest (0 bytes transferred!).
LaunchResult start_from_cache(const char* sha256_hex);

// Requests immediate cooperative abort (takes <= 20 ms even during sleep_ms).
void stop();

// Pins or unpins a cached applet in DRAM (prevents eviction).
bool set_pinned(const char* sha256_hex, bool pinned);

// Clears all unpinned entries from DRAM cache.
void clear_cache();

// Snapshot of runtime & cache statistics (safe from any task).
RuntimeStats stats();

// Writes JSON status (running, active name/sha, last result, logs) into buf.
size_t format_status_json(char* buf, size_t cap);

// Writes JSON cache inventory + Cost-Aware W-TinyLFU scores into buf.
size_t format_cache_json(char* buf, size_t cap);

// Prints status/cache summary to stdout (for `prog` console command).
void print_status();

}  // namespace prog_vm
