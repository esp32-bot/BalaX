#include "cpu_stats.h"

#include <algorithm>
#include <cmath>

#include "esp_timer.h"
#include "freertos/task.h"

namespace {

constexpr float PERCENT = 100;

}  // namespace

CpuStats::CpuStats() : mhz_(NAN) {
    for (auto& load : load_percent_) {
        load.store(NAN);
    }
}

void CpuStats::sample() {
    const int64_t now_us = esp_timer_get_time();
    const esp_cpu_cycle_count_t cycles = esp_cpu_get_cycle_count();
    const float elapsed_us = now_us - last_us_;
    if (last_us_ != 0) {
        // The cycle counter runs at the CPU clock, idle or not: cycles per microsecond is MHz.
        mhz_.store(static_cast<esp_cpu_cycle_count_t>(cycles - last_cycles_) / elapsed_us, std::memory_order_relaxed);
    }
    for (BaseType_t core = 0; core < portNUM_PROCESSORS; ++core) {
        // How long the core's idle task ran, in microseconds.
        const configRUN_TIME_COUNTER_TYPE idle_us = ulTaskGetIdleRunTimeCounterForCore(core);
        if (last_us_ != 0) {
            const float idle = static_cast<configRUN_TIME_COUNTER_TYPE>(idle_us - last_idle_us_[core]) / elapsed_us;
            load_percent_[core].store(PERCENT * std::clamp(1 - idle, 0.0f, 1.0f), std::memory_order_relaxed);
        }
        last_idle_us_[core] = idle_us;
    }
    last_us_ = now_us;
    last_cycles_ = cycles;
}
