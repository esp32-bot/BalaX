#include <cmath>
#include <vector>

#include "check.h"
#include "telemetry_stream.h"

namespace {

TEST(bxz1_compression_roundtrip_and_ratio) {
    telemetry_stream::stop_and_free();
    std::vector<robot::TraceSample> input(50);
    for (size_t i = 0; i < input.size(); ++i) {
        input[i] = {
            .tick = static_cast<uint32_t>(1000 + i),
            .state = balance::State::BALANCING,
            .fault = balance::Fault::NONE,
            .motor_l = static_cast<int8_t>(20 + (i % 7)),
            .motor_r = static_cast<int8_t>(21 + (i % 7)),
            .pitch_deg = 1.50f + 0.05f * static_cast<float>(i),
            .pitch_dps = -12.3f + 0.4f * static_cast<float>(i),
            .angle_i = 1500.0f + 2.0f * static_cast<float>(i),
            .rate_i = 15.2f,
            .speed = -10.25f,
            .distance = -1500.0f - 2.0f * static_cast<float>(i),
            .power = 42.5f,
            .accel_x_g = -0.12f + 0.01f * static_cast<float>(i % 5),
            .accel_y_g = 0.98f,
            .accel_z_g = 0.04f + 0.01f * static_cast<float>(i % 3),
            .gyro_x_dps = -12.3f + 0.4f * static_cast<float>(i),
            .gyro_y_dps = 3.2f,
            .gyro_z_dps = -1.8f,
            .yaw_deg = 15.4f + 0.2f * static_cast<float>(i),
        };
    }

    uint8_t compressed[2048] = {};
    const size_t enc_bytes =
        telemetry_stream::encode_bxz1_block(input.data(), input.size(), compressed, sizeof compressed);
    CHECK(enc_bytes > 16);
    // 50 raw TraceSamples = 50 * 64 = 3200 bytes; BXZ1 delta+varint compresses smooth 100Hz data below 1100 bytes
    CHECK(enc_bytes < 1100);

    std::vector<robot::TraceSample> decoded(50);
    const size_t dec_count =
        telemetry_stream::decode_bxz1_block(compressed, enc_bytes, decoded.data(), decoded.size());
    CHECK(dec_count == input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        CHECK(decoded[i].tick == input[i].tick);
        CHECK(decoded[i].state == input[i].state);
        CHECK(decoded[i].fault == input[i].fault);
        CHECK(decoded[i].motor_l == input[i].motor_l);
        CHECK(decoded[i].motor_r == input[i].motor_r);
        CHECK(std::abs(decoded[i].pitch_deg - input[i].pitch_deg) < 0.02f);
        CHECK(std::abs(decoded[i].pitch_dps - input[i].pitch_dps) < 0.15f);
        CHECK(std::abs(decoded[i].power - input[i].power) < 0.15f);
        CHECK(std::abs(decoded[i].accel_x_g - input[i].accel_x_g) < 0.02f);
        CHECK(std::abs(decoded[i].accel_y_g - input[i].accel_y_g) < 0.02f);
        CHECK(std::abs(decoded[i].accel_z_g - input[i].accel_z_g) < 0.02f);
        CHECK(std::abs(decoded[i].gyro_x_dps - input[i].gyro_x_dps) < 0.15f);
        CHECK(std::abs(decoded[i].gyro_y_dps - input[i].gyro_y_dps) < 0.15f);
        CHECK(std::abs(decoded[i].gyro_z_dps - input[i].gyro_z_dps) < 0.15f);
        CHECK(std::abs(decoded[i].yaw_deg - input[i].yaw_deg) < 0.15f);
    }
}

TEST(telemetry_stream_queue_ack_frees_ram) {
    telemetry_stream::stop_and_free();

    // Not active yet: pushing samples does not allocate or buffer
    for (uint32_t t = 1; t <= 60; ++t) {
        telemetry_stream::push_sample({.tick = t, .state = balance::State::READY});
    }
    CHECK(!telemetry_stream::stats().active);
    CHECK(telemetry_stream::stats().ram_chunks == 0);

    // First pull activates streaming
    uint8_t out[4096];
    uint32_t start_t = 0, end_t = 0;
    size_t n = telemetry_stream::pull_compressed(0, out, sizeof out, start_t, end_t);
    CHECK(n == 0);
    CHECK(telemetry_stream::stats().active);

    // Push 75 samples while active (1 full block of 50 + 25 staged)
    for (uint32_t t = 101; t <= 175; ++t) {
        telemetry_stream::push_sample({.tick = t, .state = balance::State::BALANCING, .pitch_deg = 2.5f});
    }
    CHECK(telemetry_stream::stats().ram_chunks == 1);

    // Pulling with ack=0 flushes the remaining 25 staged samples -> 2 chunks total
    n = telemetry_stream::pull_compressed(0, out, sizeof out, start_t, end_t);
    CHECK(n > 0);
    CHECK(start_t == 101);
    CHECK(end_t == 175);
    CHECK(telemetry_stream::stats().ram_chunks == 2);

    // Pulling with ack=150 frees the first chunk (ticks 101..150), leaving only 151..175
    n = telemetry_stream::pull_compressed(150, out, sizeof out, start_t, end_t);
    CHECK(n > 0);
    CHECK(start_t == 151);
    CHECK(end_t == 175);
    CHECK(telemetry_stream::stats().ram_chunks == 1);

    // Pulling with ack=175 frees all chunks
    n = telemetry_stream::pull_compressed(175, out, sizeof out, start_t, end_t);
    CHECK(n == 0);
    CHECK(telemetry_stream::stats().ram_chunks == 0);
    CHECK(telemetry_stream::stats().ram_bytes == 0);

    telemetry_stream::stop_and_free();
}

}  // namespace
