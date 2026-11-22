#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "micro_bios.h"
#include "prog_vm_core.h"
#include "sha256_hmac.h"

namespace {

struct MockHost final : public bxvm::Host {
    bool abort_now = false;
    int abort_after_sleeps = -1;
    int sleep_calls = 0;
    int beep_calls = 0;
    int last_beep_hz = 0;
    int last_beep_ms = 0;
    bool led_state = false;
    int last_fwd = 0;
    int last_turn = 0;
    int lcd_bars = 0;
    std::string last_lcd_text;
    std::vector<std::pair<int32_t, int32_t>> logs;

    bool should_abort() const override { return abort_now; }
    void beep(int32_t hz, int32_t ms) override { ++beep_calls; last_beep_hz = hz; last_beep_ms = ms; }
    void led(bool on) override { led_state = on; }
    bool sleep_ms(int32_t) override {
        ++sleep_calls;
        if (abort_after_sleeps >= 0 && sleep_calls >= abort_after_sleeps) abort_now = true;
        return abort_now;
    }
    void set_stick(int32_t fwd_pct, int32_t turn_pct) override { last_fwd = fwd_pct; last_turn = turn_pct; }
    void arm() override {}
    void disarm() override {}
    void takeoff() override {}
    int32_t state() const override { return 3; }
    int32_t pitch_cdeg() const override { return 125; }
    int32_t roll_cdeg() const override { return -50; }
    int32_t yaw_cdeg() const override { return 9000; }
    int32_t battery_pct() const override { return 84; }
    int32_t pmic_dc() const override { return 385; }
    int32_t uptime_ms() const override { return 1234; }
    void lcd_clear(uint16_t) override {}
    void lcd_text(int32_t, int32_t, const char* text, uint16_t, int32_t) override { last_lcd_text = text ? text : ""; }
    void lcd_bar(int32_t, int32_t, uint16_t) override { ++lcd_bars; }
    void log(int32_t code, int32_t value) override { logs.emplace_back(code, value); }
};

void emit_u16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v & 0xFF));
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void emit_i32(std::vector<uint8_t>& b, int32_t v) {
    const uint32_t u = static_cast<uint32_t>(v);
    b.push_back(static_cast<uint8_t>(u & 0xFF));
    b.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    b.push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
    b.push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
}

std::vector<uint8_t> pack_bxp(const std::vector<uint8_t>& code, const std::string& strings = "") {
    std::vector<uint8_t> out = {'B', 'X', 'P', '1', 0, 0};
    emit_u16(out, static_cast<uint16_t>(code.size()));
    emit_u16(out, static_cast<uint16_t>(strings.size()));
    out.insert(out.end(), code.begin(), code.end());
    out.insert(out.end(), strings.begin(), strings.end());
    return out;
}

}  // namespace

TEST(crypto_sha256_and_hmac_rfc4231) {
    // NIST SHA-256("abc")
    const auto d = crypto::sha256(reinterpret_cast<const uint8_t*>("abc"), 3);
    char hex[65];
    crypto::to_hex(d.data(), d.size(), hex);
    CHECK(std::strcmp(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);

    // RFC 4231 Test Case 2: key = "Jefe", data = "what do ya want for nothing?"
    const char* key = "Jefe";
    const char* msg = "what do ya want for nothing?";
    crypto::HmacSha256 h;
    h.init(reinterpret_cast<const uint8_t*>(key), 4);
    h.update(reinterpret_cast<const uint8_t*>(msg), std::strlen(msg));
    uint8_t mac[32];
    h.final(mac);
    crypto::to_hex(mac, 32, hex);
    CHECK(std::strcmp(hex, "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843") == 0);
}

TEST(micro_bios_font_and_bar) {
    int rect_calls = 0;
    int fg_pixels = 0;
    auto cb = [](void* ctx, int, int, int w, int h, uint16_t col) {
        auto* pair = static_cast<std::pair<int*, int*>*>(ctx);
        ++(*pair->first);
        if (col == BIOS_COL_GREEN) *pair->second += w * h;
    };
    std::pair<int*, int*> ctx{&rect_calls, &fg_pixels};
    bios_draw_text(cb, &ctx, 0, 0, "BALAX 100%", BIOS_COL_GREEN, BIOS_COL_BG, 2);
    CHECK(rect_calls > 20);
    CHECK(fg_pixels > 0);
    // Lowercase folds to uppercase identically
    CHECK(bios_glyph_3x5('a') == bios_glyph_3x5('A'));
    CHECK(bios_glyph_3x5('z') == bios_glyph_3x5('Z'));

    int bar_rects = 0;
    int bar_fg = 0;
    std::pair<int*, int*> bctx{&bar_rects, &bar_fg};
    bios_draw_bar(cb, &bctx, 8, 20, 102, 12, 50, BIOS_COL_GREEN, BIOS_COL_BAR_BG);
    CHECK(bar_rects >= 6);  // 4 border edges + filled + unfilled
    CHECK(bar_fg > 500);
}

TEST(bxvm_loop_syscalls_and_logs) {
    // Program: sum i=1..5 -> 15; log(1, sum); log(2, battery_pct()); beep(880, 50); lcd_text(0,0,"HI",WHITE,2); halt
    std::vector<uint8_t> c;
    // var0 = 0 (sum), var1 = 1 (i)
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::STORE_VAR)); c.push_back(0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::STORE_VAR)); c.push_back(1);

    const uint16_t loop_pc = static_cast<uint16_t>(c.size());
    // if i > 5 jump end
    c.push_back(static_cast<uint8_t>(bxvm::Op::LOAD_VAR)); c.push_back(1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 5);
    c.push_back(static_cast<uint8_t>(bxvm::Op::CMP_LE));
    c.push_back(static_cast<uint8_t>(bxvm::Op::JZ));
    const size_t patch_end = c.size();
    emit_u16(c, 0);

    // sum += i
    c.push_back(static_cast<uint8_t>(bxvm::Op::LOAD_VAR)); c.push_back(0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::LOAD_VAR)); c.push_back(1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::ADD));
    c.push_back(static_cast<uint8_t>(bxvm::Op::STORE_VAR)); c.push_back(0);
    // i += 1
    c.push_back(static_cast<uint8_t>(bxvm::Op::LOAD_VAR)); c.push_back(1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::ADD));
    c.push_back(static_cast<uint8_t>(bxvm::Op::STORE_VAR)); c.push_back(1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::JMP)); emit_u16(c, loop_pc);

    const uint16_t end_pc = static_cast<uint16_t>(c.size());
    c[patch_end] = static_cast<uint8_t>(end_pc & 0xFF);
    c[patch_end + 1] = static_cast<uint8_t>((end_pc >> 8) & 0xFF);

    // log(1, sum)
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 1);
    c.push_back(static_cast<uint8_t>(bxvm::Op::LOAD_VAR)); c.push_back(0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::SYSCALL)); c.push_back(static_cast<uint8_t>(bxvm::Syscall::LOG));
    c.push_back(static_cast<uint8_t>(bxvm::Op::POP));

    // log(2, battery_pct())
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 2);
    c.push_back(static_cast<uint8_t>(bxvm::Op::SYSCALL)); c.push_back(static_cast<uint8_t>(bxvm::Syscall::GET_BATTERY_PCT));
    c.push_back(static_cast<uint8_t>(bxvm::Op::SYSCALL)); c.push_back(static_cast<uint8_t>(bxvm::Syscall::LOG));
    c.push_back(static_cast<uint8_t>(bxvm::Op::POP));

    // beep(880, 50)
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 880);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 50);
    c.push_back(static_cast<uint8_t>(bxvm::Op::SYSCALL)); c.push_back(static_cast<uint8_t>(bxvm::Syscall::BEEP));
    c.push_back(static_cast<uint8_t>(bxvm::Op::POP));

    // lcd_text(0, 0, str0="HI", 0xFFFF, 2)
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 0xFFFF);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 2);
    c.push_back(static_cast<uint8_t>(bxvm::Op::SYSCALL)); c.push_back(static_cast<uint8_t>(bxvm::Syscall::LCD_TEXT));
    c.push_back(static_cast<uint8_t>(bxvm::Op::POP));
    c.push_back(static_cast<uint8_t>(bxvm::Op::HALT));

    const std::string strs("HI\0", 3);
    const auto bxp = pack_bxp(c, strs);
    MockHost host;
    const auto res = bxvm::execute(bxp.data(), bxp.size(), host);
    CHECK(res.ok);
    CHECK(!res.aborted);
    CHECK(host.logs.size() == 2);
    CHECK(host.logs[0].first == 1 && host.logs[0].second == 15);
    CHECK(host.logs[1].first == 2 && host.logs[1].second == 84);
    CHECK(host.beep_calls == 1 && host.last_beep_hz == 880 && host.last_beep_ms == 50);
    CHECK(host.last_lcd_text == "HI");
}

TEST(bxvm_traps_and_abort) {
    // Division by zero traps safely
    std::vector<uint8_t> c;
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 10);
    c.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(c, 0);
    c.push_back(static_cast<uint8_t>(bxvm::Op::DIV));
    const auto bxp = pack_bxp(c);
    MockHost host;
    const auto res = bxvm::execute(bxp.data(), bxp.size(), host);
    CHECK(!res.ok);
    CHECK(res.trap && std::strcmp(res.trap, "div_zero") == 0);

    // Cooperative abort during infinite loop
    std::vector<uint8_t> inf;
    inf.push_back(static_cast<uint8_t>(bxvm::Op::PUSH_I32)); emit_i32(inf, 10);
    inf.push_back(static_cast<uint8_t>(bxvm::Op::SYSCALL)); inf.push_back(static_cast<uint8_t>(bxvm::Syscall::SLEEP_MS));
    inf.push_back(static_cast<uint8_t>(bxvm::Op::POP));
    inf.push_back(static_cast<uint8_t>(bxvm::Op::JMP)); emit_u16(inf, 0);
    const auto bxp2 = pack_bxp(inf);
    MockHost host2;
    host2.abort_after_sleeps = 3;
    const auto res2 = bxvm::execute(bxp2.data(), bxp2.size(), host2);
    CHECK(!res2.ok);
    CHECK(res2.aborted);
    CHECK(host2.sleep_calls == 3);
}

TEST(dram_applet_cache_wtinylfu_and_pin) {
    bxvm::DramAppletCache cache(300);
    std::vector<uint8_t> a(100, 0x11), b(100, 0x22), c(100, 0x33), d(120, 0x44);

    // Insert A (pinned), B (frequently accessed), C (cold)
    CHECK(cache.put("aaa", "A", a.data(), a.size(), true));
    CHECK(cache.put("bbb", "B", b.data(), b.size(), false));
    CHECK(cache.put("ccc", "C", c.data(), c.size(), false));
    // Touch B multiple times so W-TinyLFU protects B over C
    CHECK(cache.get("bbb") != nullptr);
    CHECK(cache.get("bbb") != nullptr);
    CHECK(cache.get("bbb") != nullptr);
    CHECK(cache.stats().hits == 3);

    // Dedup check: re-putting B must not grow used_bytes or evict anything
    CHECK(cache.put("bbb", "B", b.data(), b.size(), false));
    CHECK(cache.stats().used_bytes == 300);
    CHECK(cache.stats().evictions == 0);

    // Inserting D (120 bytes) needs to evict C (cold) and NOT A (pinned) or B (hot)!
    // Note: 300 - 100 + 120 = 320 > 300, so both C and B would need eviction unless D <= 100.
    // Test with D = 100 bytes so exactly ONE victim (C) is evicted!
    std::vector<uint8_t> d100(100, 0x44);
    CHECK(cache.put("ddd", "D", d100.data(), d100.size(), false));
    CHECK(cache.stats().evictions == 1);
    CHECK(cache.get("aaa") != nullptr);  // Pinned preserved
    CHECK(cache.get("bbb") != nullptr);  // Hot preserved
    CHECK(cache.get("ddd") != nullptr);  // Newly inserted present
    CHECK(cache.get("ccc") == nullptr);  // Cold evicted!

    // Clear unpinned preserves only pinned A
    cache.clear_unpinned();
    CHECK(cache.stats().entries == 1);
    CHECK(cache.get("aaa") != nullptr);
}
