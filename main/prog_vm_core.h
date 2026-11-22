#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace bxvm {

// BXP1 binary header: [magic "BXP1" (4)][flags (1)][reserved (1)][code_len (2)][str_len (2)] = 10 bytes
inline constexpr uint8_t MAGIC[4] = {'B', 'X', 'P', '1'};
inline constexpr size_t HEADER_BYTES = 10;
inline constexpr size_t MAX_PROG_BYTES = 16 * 1024;
inline constexpr size_t DRAM_CACHE_BUDGET_BYTES = 24 * 1024;
inline constexpr size_t MAX_STACK = 64;
inline constexpr size_t MAX_VARS = 32;
inline constexpr uint32_t MAX_STEPS_PER_RUN = 200'000;

enum class Op : uint8_t {
    HALT      = 0x00,
    PUSH_I32  = 0x01,  // followed by int32_t little-endian
    POP       = 0x02,
    DUP       = 0x03,
    ADD       = 0x10,
    SUB       = 0x11,
    MUL       = 0x12,
    DIV       = 0x13,
    MOD       = 0x14,
    NEG       = 0x15,
    CMP_EQ    = 0x20,
    CMP_NE    = 0x21,
    CMP_LT    = 0x22,
    CMP_LE    = 0x23,
    CMP_GT    = 0x24,
    CMP_GE    = 0x25,
    JMP       = 0x30,  // followed by uint16_t pc
    JZ        = 0x31,  // pop; if 0 jump to uint16_t pc
    JNZ       = 0x32,  // pop; if !=0 jump to uint16_t pc
    LOAD_VAR  = 0x40,  // followed by uint8_t idx (0..31)
    STORE_VAR = 0x41,  // followed by uint8_t idx (0..31)
    SYSCALL   = 0x50,  // followed by uint8_t syscall_id
};

enum class Syscall : uint8_t {
    BEEP            = 0x01,  // (hz, ms) -> 0
    LED             = 0x02,  // (on) -> 0
    SLEEP_MS        = 0x03,  // (ms) -> aborted (0/1)
    SET_STICK       = 0x04,  // (fwd_pct, turn_pct) -> 0
    ARM             = 0x05,  // () -> 0
    DISARM          = 0x06,  // () -> 0
    TAKEOFF         = 0x07,  // () -> 0
    GET_STATE       = 0x08,  // () -> state
    GET_PITCH_CDEG  = 0x09,  // () -> pitch*100
    GET_ROLL_CDEG   = 0x0A,  // () -> roll*100
    GET_YAW_CDEG    = 0x0B,  // () -> yaw*100
    GET_BATTERY_PCT = 0x0C,  // () -> pct
    GET_PMIC_DC     = 0x0D,  // () -> pmic_c*10
    GET_UPTIME_MS   = 0x0E,  // () -> ms
    LCD_CLEAR       = 0x0F,  // (rgb565) -> 0
    LCD_TEXT        = 0x10,  // (x, y, str_idx, fg, scale) -> 0
    LCD_BAR         = 0x11,  // (y, pct, fg) -> 0
    LOG             = 0x12,  // (code, val) -> 0
};

struct Host {
    virtual ~Host() = default;
    virtual bool should_abort() const = 0;
    virtual void beep(int32_t hz, int32_t ms) = 0;
    virtual void led(bool on) = 0;
    virtual bool sleep_ms(int32_t ms) = 0;
    virtual void set_stick(int32_t fwd_pct, int32_t turn_pct) = 0;
    virtual void arm() = 0;
    virtual void disarm() = 0;
    virtual void takeoff() = 0;
    virtual int32_t state() const = 0;
    virtual int32_t pitch_cdeg() const = 0;
    virtual int32_t roll_cdeg() const = 0;
    virtual int32_t yaw_cdeg() const = 0;
    virtual int32_t battery_pct() const = 0;
    virtual int32_t pmic_dc() const = 0;
    virtual int32_t uptime_ms() const = 0;
    virtual void lcd_clear(uint16_t rgb565) = 0;
    virtual void lcd_text(int32_t x, int32_t y, const char* text, uint16_t fg, int32_t scale) = 0;
    virtual void lcd_bar(int32_t y, int32_t pct, uint16_t fg) = 0;
    virtual void log(int32_t code, int32_t value) = 0;
};

struct RunResult {
    bool ok = false;
    bool aborted = false;
    uint32_t steps = 0;
    const char* trap = nullptr;
};

inline uint16_t read_u16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

inline int32_t read_i32(const uint8_t* p) {
    const uint32_t u = static_cast<uint32_t>(p[0]) |
                       (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) |
                       (static_cast<uint32_t>(p[3]) << 24);
    return static_cast<int32_t>(u);
}

inline const char* get_string(const uint8_t* str_table, size_t str_len, int32_t idx) {
    if (idx < 0 || !str_table || str_len == 0) return "";
    size_t pos = 0;
    int32_t cur = 0;
    while (pos < str_len) {
        if (cur == idx) {
            return reinterpret_cast<const char*>(str_table + pos);
        }
        while (pos < str_len && str_table[pos] != '\0') ++pos;
        if (pos < str_len) ++pos;
        ++cur;
    }
    return "";
}

inline RunResult execute(const uint8_t* prog, size_t prog_len, Host& host) {
    RunResult r{};
    if (!prog || prog_len < HEADER_BYTES || prog_len > MAX_PROG_BYTES) {
        r.trap = "invalid_size";
        return r;
    }
    if (std::memcmp(prog, MAGIC, 4) != 0) {
        r.trap = "bad_magic";
        return r;
    }
    const uint16_t code_len = read_u16(prog + 6);
    const uint16_t str_len = read_u16(prog + 8);
    if (HEADER_BYTES + code_len + str_len > prog_len) {
        r.trap = "truncated";
        return r;
    }
    const uint8_t* code = prog + HEADER_BYTES;
    const uint8_t* strings = code + code_len;
    if (str_len > 0 && strings[str_len - 1] != '\0') {
        r.trap = "unterminated_strings";
        return r;
    }

    int32_t stack[MAX_STACK] = {};
    int32_t vars[MAX_VARS] = {};
    size_t sp = 0;
    size_t pc = 0;

    auto push = [&](int32_t v) -> bool {
        if (sp >= MAX_STACK) { r.trap = "stack_overflow"; return false; }
        stack[sp++] = v;
        return true;
    };
    auto pop = [&](int32_t& out) -> bool {
        if (sp == 0) { r.trap = "stack_underflow"; return false; }
        out = stack[--sp];
        return true;
    };

    while (pc < code_len) {
        if (host.should_abort()) {
            r.aborted = true;
            r.trap = "aborted";
            return r;
        }
        if (++r.steps > MAX_STEPS_PER_RUN) {
            r.trap = "step_limit";
            return r;
        }
        const Op op = static_cast<Op>(code[pc++]);
        switch (op) {
            case Op::HALT:
                r.ok = true;
                return r;
            case Op::PUSH_I32:
                if (pc + 4 > code_len) { r.trap = "oob_imm"; return r; }
                if (!push(read_i32(code + pc))) return r;
                pc += 4;
                break;
            case Op::POP: {
                int32_t d = 0;
                if (!pop(d)) return r;
                break;
            }
            case Op::DUP:
                if (sp == 0) { r.trap = "stack_underflow"; return r; }
                if (!push(stack[sp - 1])) return r;
                break;
            case Op::ADD:
            case Op::SUB:
            case Op::MUL:
            case Op::DIV:
            case Op::MOD:
            case Op::CMP_EQ:
            case Op::CMP_NE:
            case Op::CMP_LT:
            case Op::CMP_LE:
            case Op::CMP_GT:
            case Op::CMP_GE: {
                int32_t b = 0, a = 0;
                if (!pop(b) || !pop(a)) return r;
                int32_t res = 0;
                if (op == Op::ADD) res = a + b;
                else if (op == Op::SUB) res = a - b;
                else if (op == Op::MUL) res = a * b;
                else if (op == Op::DIV) { if (b == 0) { r.trap = "div_zero"; return r; } res = a / b; }
                else if (op == Op::MOD) { if (b == 0) { r.trap = "div_zero"; return r; } res = a % b; }
                else if (op == Op::CMP_EQ) res = (a == b);
                else if (op == Op::CMP_NE) res = (a != b);
                else if (op == Op::CMP_LT) res = (a < b);
                else if (op == Op::CMP_LE) res = (a <= b);
                else if (op == Op::CMP_GT) res = (a > b);
                else if (op == Op::CMP_GE) res = (a >= b);
                if (!push(res)) return r;
                break;
            }
            case Op::NEG: {
                int32_t a = 0;
                if (!pop(a) || !push(-a)) return r;
                break;
            }
            case Op::JMP:
            case Op::JZ:
            case Op::JNZ: {
                if (pc + 2 > code_len) { r.trap = "oob_jmp"; return r; }
                const uint16_t target = read_u16(code + pc);
                pc += 2;
                if (target > code_len) { r.trap = "bad_jmp_target"; return r; }
                bool take = (op == Op::JMP);
                if (!take) {
                    int32_t cond = 0;
                    if (!pop(cond)) return r;
                    take = (op == Op::JZ) ? (cond == 0) : (cond != 0);
                }
                if (take) pc = target;
                break;
            }
            case Op::LOAD_VAR: {
                if (pc >= code_len) { r.trap = "oob_var"; return r; }
                const uint8_t idx = code[pc++];
                if (idx >= MAX_VARS) { r.trap = "bad_var"; return r; }
                if (!push(vars[idx])) return r;
                break;
            }
            case Op::STORE_VAR: {
                if (pc >= code_len) { r.trap = "oob_var"; return r; }
                const uint8_t idx = code[pc++];
                if (idx >= MAX_VARS) { r.trap = "bad_var"; return r; }
                int32_t v = 0;
                if (!pop(v)) return r;
                vars[idx] = v;
                break;
            }
            case Op::SYSCALL: {
                if (pc >= code_len) { r.trap = "oob_syscall"; return r; }
                const Syscall sc = static_cast<Syscall>(code[pc++]);
                int32_t a = 0, b = 0, c = 0, d = 0, e = 0;
                switch (sc) {
                    case Syscall::BEEP:
                        if (!pop(b) || !pop(a)) return r;
                        host.beep(a, b);
                        if (!push(0)) return r;
                        break;
                    case Syscall::LED:
                        if (!pop(a)) return r;
                        host.led(a != 0);
                        if (!push(0)) return r;
                        break;
                    case Syscall::SLEEP_MS:
                        if (!pop(a)) return r;
                        if (!push(host.sleep_ms(a) ? 1 : 0)) return r;
                        break;
                    case Syscall::SET_STICK:
                        if (!pop(b) || !pop(a)) return r;
                        host.set_stick(std::clamp<int32_t>(a, -100, 100), std::clamp<int32_t>(b, -100, 100));
                        if (!push(0)) return r;
                        break;
                    case Syscall::ARM:     host.arm();     if (!push(0)) return r; break;
                    case Syscall::DISARM:  host.disarm();  if (!push(0)) return r; break;
                    case Syscall::TAKEOFF: host.takeoff(); if (!push(0)) return r; break;
                    case Syscall::GET_STATE:       if (!push(host.state())) return r; break;
                    case Syscall::GET_PITCH_CDEG:  if (!push(host.pitch_cdeg())) return r; break;
                    case Syscall::GET_ROLL_CDEG:   if (!push(host.roll_cdeg())) return r; break;
                    case Syscall::GET_YAW_CDEG:    if (!push(host.yaw_cdeg())) return r; break;
                    case Syscall::GET_BATTERY_PCT: if (!push(host.battery_pct())) return r; break;
                    case Syscall::GET_PMIC_DC:     if (!push(host.pmic_dc())) return r; break;
                    case Syscall::GET_UPTIME_MS:   if (!push(host.uptime_ms())) return r; break;
                    case Syscall::LCD_CLEAR:
                        if (!pop(a)) return r;
                        host.lcd_clear(static_cast<uint16_t>(a));
                        if (!push(0)) return r;
                        break;
                    case Syscall::LCD_TEXT:
                        if (!pop(e) || !pop(d) || !pop(c) || !pop(b) || !pop(a)) return r;
                        host.lcd_text(a, b, get_string(strings, str_len, c), static_cast<uint16_t>(d), e);
                        if (!push(0)) return r;
                        break;
                    case Syscall::LCD_BAR:
                        if (!pop(c) || !pop(b) || !pop(a)) return r;
                        host.lcd_bar(a, b, static_cast<uint16_t>(c));
                        if (!push(0)) return r;
                        break;
                    case Syscall::LOG:
                        if (!pop(b) || !pop(a)) return r;
                        host.log(a, b);
                        if (!push(0)) return r;
                        break;
                    default:
                        r.trap = "unknown_syscall";
                        return r;
                }
                break;
            }
            default:
                r.trap = "unknown_op";
                return r;
        }
    }
    r.ok = true;
    return r;
}

// Tier-0 Zero-Flash-Wear DRAM Applet Cache with Cost-Aware W-TinyLFU Eviction.
struct CacheEntry {
    std::string sha256_hex;
    std::string name;
    std::vector<uint8_t> bytes;
    float freq_decayed = 1.0f;
    float refetch_cost = 1.0f;
    uint64_t last_access_seq = 0;
    bool pinned = false;
};

struct CacheStats {
    uint32_t hits = 0;
    uint32_t misses = 0;
    uint32_t evictions = 0;
    size_t used_bytes = 0;
    size_t budget_bytes = DRAM_CACHE_BUDGET_BYTES;
    size_t entries = 0;
};

class DramAppletCache {
public:
    explicit DramAppletCache(size_t budget = DRAM_CACHE_BUDGET_BYTES) : budget_(budget) {}

    const CacheEntry* get(const std::string& sha256_hex) {
        for (auto& e : entries_) {
            if (e.sha256_hex == sha256_hex) {
                ++seq_;
                decay_if_needed();
                e.freq_decayed += 1.0f;
                e.last_access_seq = seq_;
                ++hits_;
                return &e;
            }
        }
        ++misses_;
        return nullptr;
    }

    bool put(const std::string& sha256_hex,
             const std::string& name,
             const uint8_t* data,
             size_t len,
             bool pinned = false,
             float refetch_cost = 1.0f) {
        if (!data || len == 0 || len > budget_) return false;
        // Dedup: if already cached, update metadata and return without reallocating
        for (auto& e : entries_) {
            if (e.sha256_hex == sha256_hex) {
                ++seq_;
                e.name = name.empty() ? e.name : name;
                e.freq_decayed += 1.0f;
                e.last_access_seq = seq_;
                if (pinned) e.pinned = true;
                e.refetch_cost = std::max(e.refetch_cost, refetch_cost);
                return true;
            }
        }
        // Evict lowest Cost-Aware W-TinyLFU score unpinned entries until budget fits
        while (used_bytes_ + len > budget_) {
            const int victim = pick_victim();
            if (victim < 0) return false;  // all remaining entries are pinned
            used_bytes_ -= entries_[victim].bytes.size();
            entries_.erase(entries_.begin() + victim);
            ++evictions_;
        }
        ++seq_;
        decay_if_needed();
        CacheEntry entry;
        entry.sha256_hex = sha256_hex;
        entry.name = name;
        entry.bytes.assign(data, data + len);
        entry.freq_decayed = 1.0f;
        entry.refetch_cost = refetch_cost;
        entry.last_access_seq = seq_;
        entry.pinned = pinned;
        used_bytes_ += len;
        entries_.push_back(std::move(entry));
        return true;
    }

    bool set_pinned(const std::string& sha256_hex, bool pinned) {
        for (auto& e : entries_) {
            if (e.sha256_hex == sha256_hex) {
                e.pinned = pinned;
                return true;
            }
        }
        return false;
    }

    void clear_unpinned() {
        entries_.erase(
            std::remove_if(entries_.begin(), entries_.end(), [&](const CacheEntry& e) {
                if (!e.pinned) {
                    used_bytes_ -= e.bytes.size();
                    ++evictions_;
                    return true;
                }
                return false;
            }),
            entries_.end());
    }

    const std::vector<CacheEntry>& entries() const { return entries_; }

    CacheStats stats() const {
        return CacheStats{hits_, misses_, evictions_, used_bytes_, budget_, entries_.size()};
    }

    float score_of(const CacheEntry& e) const {
        if (e.pinned) return 1e30f;
        const float age = static_cast<float>(std::max<uint64_t>(1, seq_ - e.last_access_seq + 1));
        const float size_kb = std::max(0.25f, static_cast<float>(e.bytes.size()) / 1024.0f);
        return (e.freq_decayed * e.refetch_cost) / (age * size_kb);
    }

private:
    void decay_if_needed() {
        if (seq_ % 32 == 0) {
            for (auto& e : entries_) {
                e.freq_decayed = std::max(0.25f, e.freq_decayed * 0.5f);
            }
        }
    }

    int pick_victim() const {
        int best_idx = -1;
        float lowest = 1e30f;
        for (size_t i = 0; i < entries_.size(); ++i) {
            if (entries_[i].pinned) continue;
            const float s = score_of(entries_[i]);
            if (best_idx < 0 || s < lowest) {
                lowest = s;
                best_idx = static_cast<int>(i);
            }
        }
        return best_idx;
    }

    size_t budget_;
    size_t used_bytes_ = 0;
    uint64_t seq_ = 0;
    uint32_t hits_ = 0;
    uint32_t misses_ = 0;
    uint32_t evictions_ = 0;
    std::vector<CacheEntry> entries_;
};

}  // namespace bxvm
