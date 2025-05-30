/*
 * Copyright (C) 2014 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 * Copyright (C) 2025 RocketGod - Added modes from my Flipper Zero RF Jammer App - https://betaskynet.com
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "proc_jammer.hpp"
#include "portapack_shared_memory.hpp"
#include "sine_table_int8.hpp"
#include "event_m4.hpp"

#include <cstdint>
#include <random>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void JammerProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured) return;

    for (size_t i = 0; i < buffer.count; i++) {
        if (jammer_duration != 0xFFFFFFFF && !jammer_duration) {
            uint32_t start_range = current_range;
            uint32_t old_wave_phase = wave_phase;
            do {
                current_range++;
                if (current_range == JAMMER_MAX_CH) current_range = 0;
                if (current_range == start_range) {
                    configured = false;
                    return;
                }
            } while (!jammer_channels[current_range].enabled);

            jammer_duration = jammer_channels[current_range].duration;
            jammer_bw = jammer_channels[current_range].width / 2;
            message.freq = jammer_channels[current_range].center;
            message.range = current_range;
            shared_memory.application_queue.push(message);
            wave_phase = old_wave_phase;  // Preserve phase for continuity
        } else if (jammer_duration != 0xFFFFFFFF) {
            jammer_duration--;
        }

        if (!period_counter) {
            period_counter = noise_period;

            if (noise_type == jammer::JammerType::TYPE_FSK) {
                sample = (lfsr & 0xFF) - 128;  // Full ±127 range, like Flipper 2FSK
            } else if (noise_type == jammer::JammerType::TYPE_TONE) {
                tone_delta = 150000 + (lfsr >> 9);  // Matches Flipper FM tone
            } else if (noise_type == jammer::JammerType::TYPE_SWEEP) {
                sample++;
            } else if (noise_type == jammer::JammerType::TYPE_RANDOM) {
                sample = (lfsr & 0xFF) - 128;  // Like Flipper WhiteNoise
            } else if (noise_type == jammer::JammerType::TYPE_SINE) {
                uint32_t phase_increment = (waveform_freq * (1ULL << 32)) / 3072000;
                wave_phase += phase_increment;
                sample = sine_table_i8[(wave_phase >> 24) & 0xFF];  // Like Flipper SineWave
            } else if (noise_type == jammer::JammerType::TYPE_SQUARE) {
                square_counter++;
                uint32_t square_period = 3072000 / (2 * waveform_freq);
                if (square_counter >= square_period) {
                    square_counter = 0;
                    wave_index = (wave_index + 1) % 2;
                }
                sample = wave_index ? 127 : -128;  // Like Flipper SquareWave
            } else if (noise_type == jammer::JammerType::TYPE_SAWTOOTH) {
                saw_counter++;
                uint32_t saw_period = 3072000 / waveform_freq;
                if (saw_counter >= saw_period) {
                    saw_counter = 0;
                    wave_index = (wave_index + 1) % 256;
                }
                sample = -128 + (wave_index * 255) / 256;  // Like Flipper SawtoothWave
            } else if (noise_type == jammer::JammerType::TYPE_TRIANGLE) {
                tri_counter++;
                uint32_t tri_period = 3072000 / waveform_freq;
                if (tri_counter >= tri_period) {
                    tri_counter = 0;
                    wave_index = (wave_index + 1) % 256;
                }
                sample = (wave_index < 128 ? wave_index * 2 : (255 - wave_index) * 2) - 128;  // Like Flipper TriangleWave
            } else if (noise_type == jammer::JammerType::TYPE_CHIRP) {
                chirp_freq += 0.01f;
                if (chirp_freq > 1.0f) chirp_freq = 0.0f;
                uint32_t base_freq = waveform_freq;
                uint32_t max_freq = waveform_freq * 2;
                uint32_t chirp_freq_scaled = base_freq + (chirp_freq * (max_freq - base_freq));
                wave_phase += (chirp_freq_scaled * (1ULL << 32)) / 3072000;
                sample = sine_table_i8[(wave_phase >> 24) & 0xFF];  // Like Flipper Chirp
            } else if (noise_type == jammer::JammerType::TYPE_GAUSSIAN) {
                float u1 = static_cast<float>(lfsr & 0xFFFF) / 0x10000;
                float u2 = static_cast<float>((lfsr >> 16) & 0xFFFF) / 0x10000;
                if (u1 == 0.0f) u1 = 1e-10f;  // Avoid log(0)
                float gaussian = std::sqrt(-2.0f * std::log(u1)) * std::cos(2 * M_PI * u2);
                sample = static_cast<int8_t>(std::max(-127.0f, std::min(127.0f, gaussian * 127.0f)));  // Like Flipper GaussianNoise
            } else if (noise_type == jammer::JammerType::TYPE_BRUTEFORCE) {
                sample = 127;  // Like Flipper Bruteforce
            }

            feedback = ((lfsr >> 31) ^ (lfsr >> 29) ^ (lfsr >> 15) ^ (lfsr >> 11)) & 1;
            lfsr = (lfsr << 1) | feedback;
            if (!lfsr) lfsr = 0x1337;
        } else {
            period_counter--;
        }

        if (noise_type == jammer::JammerType::TYPE_TONE) {
            aphase += tone_delta;
            sample = sine_table_i8[(aphase & 0xFF000000) >> 24];
        }

        delta = sample * jammer_bw;
        phase += delta;
        sphase = phase + (64 << 24);
        re = sine_table_i8[(sphase & 0xFF000000) >> 24];
        im = sine_table_i8[(phase & 0xFF000000) >> 24];
        buffer.p[i] = {re, im};
    }
}

void JammerProcessor::on_message(const Message* const msg) {
    if (msg->id == Message::ID::JammerConfigure) {
        const auto message = *reinterpret_cast<const JammerConfigureMessage*>(msg);
        if (message.run) {
            jammer_channels = (JammerChannel*)shared_memory.bb_data.data;
            noise_type = message.type;
            noise_period = 3072000 / message.speed;
            waveform_freq = message.waveform_freq;
            if (noise_type == jammer::JammerType::TYPE_SWEEP || noise_type == jammer::JammerType::TYPE_SINE ||
                noise_type == jammer::JammerType::TYPE_SQUARE || noise_type == jammer::JammerType::TYPE_SAWTOOTH ||
                noise_type == jammer::JammerType::TYPE_TRIANGLE || noise_type == jammer::JammerType::TYPE_CHIRP ||
                noise_type == jammer::JammerType::TYPE_GAUSSIAN || noise_type == jammer::JammerType::TYPE_BRUTEFORCE) {
                noise_period >>= 8;
                if (noise_period < 1) noise_period = 1;
            }
            period_counter = 0;
            // Preserve channel state, only reset if not configured
            if (!configured) {
                jammer_duration = 0;
                current_range = 0;
            }
            lfsr = 0xDEAD0012;
            wave_phase = 0;  // Reset for new mode/frequency
            wave_index = 0;
            square_counter = 0;
            saw_counter = 0;
            tri_counter = 0;
            chirp_freq = 0.0f;
            configured = true;
        } else {
            configured = false;
        }
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<JammerProcessor>()};
    event_dispatcher.run();
    return 0;
}
