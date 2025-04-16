/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2023 RocketGod
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

#ifndef __PROC_FLEX_H__
#define __PROC_FLEX_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"

#include "dsp_decimate.hpp"
#include "dsp_demodulate.hpp"

#include "audio_output.hpp"

#include "fifo.hpp"
#include "message.hpp"

// FLEX sync patterns
#define SYNC1           0xA6C6
#define SYNC2           0xAAAA
#define EOT1            0xAAAA
#define EOT2            0xFFFF

// BCH error correction polynomials
#define BCH_POLY        0x769  // 1110110101001 (x^11 + x^9 + x^8 + x^7 + x^5 + x^3 + 1)
#define BCH_N           31
#define BCH_K           21
#define BCH_T           2      // Error correction capability (2 bits)

// FLEX message types
#define TYPE_SECURE     0
#define TYPE_SHORT_INSTR 1
#define TYPE_TONE       2
#define TYPE_STANDARD_NUM 3
#define TYPE_SPECIAL_NUM  4
#define TYPE_ALPHANUMERIC 5
#define TYPE_BINARY     6
#define TYPE_NUMBERED_NUM 7

// FLEX baud rates
#define FLEX_BAUD_1600  1600
#define FLEX_BAUD_3200  3200
#define FLEX_BAUD_6400  6400

class FlexProcessor : public BasebandProcessor {
public:
    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

private:
    static constexpr size_t baseband_fs = 3072000;
    static constexpr size_t audio_fs = baseband_fs / 8 / 8 / 2;

    // Signal buffers
    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{dst.data(), dst.size()};
    std::array<float, 32> audio{};
    const buffer_f32_t audio_buffer{audio.data(), audio.size()};
    
    // DSP chain
    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};
    dsp::demodulate::FM demod{};
    AudioOutput audio_output{};
    
    // FSK signal processing
    std::array<int32_t, 64> delay_line{0};
    size_t delay_line_index{0};
    int32_t sample_mixed{0};
    int32_t prev_mixed{0};
    int32_t sample_filtered{0};
    int32_t prev_filtered{0};
    uint32_t sample_bits{0};
    
    // FLEX sync detection
    uint16_t sync_buffer[4]{0};
    int sync_counter{0};
    int bit_counter{0};
    int phase{0};
    double phase_inc{0.0};
    
    // Frame processing
    bool sync_found{false};
    uint16_t baud_rate{FLEX_BAUD_1600};
    uint8_t fsk_levels{2};  // 2 or 4 level FSK
    
    // FLEX frame state
    uint8_t current_frame{0};
    uint8_t current_cycle{0};
    
    // Frame data storage
    flex_batch_t batch{};
    size_t batch_index{0};
    
    // BCH error correction
    uint32_t bch_check(uint32_t codeword);
    uint32_t bch_correct(uint32_t codeword);
    
    // Bit manipulation helpers
    uint32_t reverse_bits(uint32_t value, int num_bits);
    int count_bits(uint32_t value);
    
    // FLEX protocol processing
    void process_flex_symbol(uint8_t symbol);
    bool detect_flex_sync();
    void decode_frame_info();
    void process_addresses_and_vectors();
    void process_bch_block(uint32_t* block, size_t count);
    
    // Configuration and status
    bool configured{false};
    uint32_t bits_processed{0};
    uint8_t frames_processed{0};
    
    void reset();
    void configure(const FlexConfigureMessage& message);
    
    // Send status updates to UI
    void send_status_update();
    
    // Timestamp & packet handling
    Timestamp timestamp{};
    void send_packet();
    
    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_FLEX_H__*/