#pragma once

// How hard the CPU works, averaged between samples: clock speed and load per core.
//
// The CPU runs at CPU_MAX_MHZ while either core is busy and drops to CPU_MIN_MHZ only while
// both are idle, so the clock speed seen by running code is always the maximum. The averages
// over time are what set power draw and heat.

#include <atomic>

#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"

class CpuStats {
public:
    CpuStats();

    // Call from one task pinned to a core (the cycle counter is per core), at least every
    // 17 s (when the counter wraps at 240 MHz).
    void sample();

    // These are NaN until there have been two samples.
    float mhz() const { return mhz_.load(std::memory_order_relaxed); }
    float load_percent(BaseType_t core) const { return load_percent_[core].load(std::memory_order_relaxed); }

private:
    int64_t last_us_ = 0;
    esp_cpu_cycle_count_t last_cycles_ = 0;
    configRUN_TIME_COUNTER_TYPE last_idle_us_[portNUM_PROCESSORS] = {};
    std::atomic<float> mhz_;
    std::atomic<float> load_percent_[portNUM_PROCESSORS];
};
