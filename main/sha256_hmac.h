#pragma once

// Minimal zero-dependency SHA-256 + HMAC-SHA256 + hex utilities.
// Compiles identically on ESP32 (avoiding mbedTLS/tfpsacrypto Flash bloat) and on host C++ tests.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace crypto {

struct Sha256 {
    uint32_t h[8] = {};
    uint8_t buf[64] = {};
    size_t buf_len = 0;
    uint64_t total_len = 0;

    static constexpr uint32_t K[64] = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
    };

    static inline uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

    void init() {
        h[0] = 0x6a09e667u; h[1] = 0xbb67ae85u; h[2] = 0x3c6ef372u; h[3] = 0xa54ff53au;
        h[4] = 0x510e527fu; h[5] = 0x9b05688cu; h[6] = 0x1f83d9abu; h[7] = 0x5be0cd19u;
        buf_len = 0;
        total_len = 0;
    }

    void compress(const uint8_t block[64]) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(block[i * 4]) << 24) |
                   (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
                   static_cast<uint32_t>(block[i * 4 + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t temp1 = hh + S1 + ch + K[i] + w[i];
            const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = S0 + maj;
            hh = g; g = f; f = e; e = d + temp1;
            d = c; c = b; b = a; a = temp1 + temp2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d;
        h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }

    void update(const uint8_t* data, size_t len) {
        if (!data || len == 0) return;
        total_len += len;
        size_t pos = 0;
        if (buf_len > 0) {
            const size_t take = (64 - buf_len < len) ? (64 - buf_len) : len;
            std::memcpy(buf + buf_len, data, take);
            buf_len += take;
            pos += take;
            if (buf_len == 64) {
                compress(buf);
                buf_len = 0;
            }
        }
        while (pos + 64 <= len) {
            compress(data + pos);
            pos += 64;
        }
        if (pos < len) {
            buf_len = len - pos;
            std::memcpy(buf, data + pos, buf_len);
        }
    }

    void final(uint8_t out[32]) {
        const uint64_t total_bits = total_len * 8;
        buf[buf_len++] = 0x80u;
        if (buf_len > 56) {
            std::memset(buf + buf_len, 0, 64 - buf_len);
            compress(buf);
            buf_len = 0;
        }
        std::memset(buf + buf_len, 0, 56 - buf_len);
        for (int i = 0; i < 8; ++i) {
            buf[56 + i] = static_cast<uint8_t>(total_bits >> (56 - i * 8));
        }
        compress(buf);
        for (int i = 0; i < 8; ++i) {
            out[i * 4]     = static_cast<uint8_t>(h[i] >> 24);
            out[i * 4 + 1] = static_cast<uint8_t>(h[i] >> 16);
            out[i * 4 + 2] = static_cast<uint8_t>(h[i] >> 8);
            out[i * 4 + 3] = static_cast<uint8_t>(h[i]);
        }
    }
};

struct HmacSha256 {
    Sha256 inner;
    uint8_t opad[64] = {};

    void init(const uint8_t* key, size_t key_len) {
        uint8_t key_block[64] = {};
        if (key_len > 64) {
            Sha256 kh;
            kh.init();
            kh.update(key, key_len);
            kh.final(key_block);
        } else if (key && key_len > 0) {
            std::memcpy(key_block, key, key_len);
        }
        uint8_t ipad[64];
        for (size_t i = 0; i < 64; ++i) {
            ipad[i] = key_block[i] ^ 0x36u;
            opad[i] = key_block[i] ^ 0x5cu;
        }
        inner.init();
        inner.update(ipad, 64);
    }

    void update(const uint8_t* data, size_t len) { inner.update(data, len); }

    void final(uint8_t out[32]) {
        uint8_t inner_hash[32];
        inner.final(inner_hash);
        Sha256 outer;
        outer.init();
        outer.update(opad, 64);
        outer.update(inner_hash, 32);
        outer.final(out);
    }
};

inline void to_hex(const uint8_t* bytes, size_t len, char* out) {
    static constexpr char HEX[] = "0123456789abcdef";
    for (size_t i = 0; i < len; ++i) {
        out[i * 2]     = HEX[bytes[i] >> 4];
        out[i * 2 + 1] = HEX[bytes[i] & 0x0Fu];
    }
    out[len * 2] = '\0';
}

inline int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

inline bool from_hex(const char* hex, uint8_t* out, size_t out_len) {
    if (!hex) return false;
    if (std::strlen(hex) != out_len * 2) return false;
    for (size_t i = 0; i < out_len; ++i) {
        const int hi = hex_nibble(hex[i * 2]);
        const int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
}

inline bool constant_time_eq(const uint8_t* a, const uint8_t* b, size_t len) {
    uint8_t diff = 0;
    for (size_t i = 0; i < len; ++i) {
        diff |= static_cast<uint8_t>(a[i] ^ b[i]);
    }
    return diff == 0;
}

inline std::array<uint8_t, 32> sha256(const uint8_t* data, size_t len) {
    Sha256 s;
    s.init();
    s.update(data, len);
    std::array<uint8_t, 32> out{};
    s.final(out.data());
    return out;
}

}  // namespace crypto
