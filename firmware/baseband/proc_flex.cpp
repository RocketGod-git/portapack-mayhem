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

#include "proc_flex.hpp"
#include "portapack_shared_memory.hpp"
#include "event_m4.hpp"
#include "audio_dma.hpp"

// Symbol translation table - maps received modem status into symbols (from PDW)
const int rcv_symbols[16] = {0, 1, 1, 2, 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3};

// FLEX sync patterns (from PDW)
const uint16_t flex_syncs[8] = {0x870C, 0x7B18, 0xB068, 0xDEA0, 0x22B4, 0xE9C4, 0x4C7C, 0x34DF};

void FlexProcessor::execute(const buffer_c8_t& buffer) {
    // This function is called at 3072000/2048 = 1500Hz

    if (!configured) return;

    // FM demodulation
    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);              // 2048/8 = 256 (512 I/Q samples)
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);         // 256/8 = 32 (64 I/Q samples)
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);  // 32/2 = 16 (32 I/Q samples)

    feed_channel_stats(channel_out);

    auto audio = demod.execute(channel_out, audio_buffer);

    audio_output.write(audio);

    // Process the audio samples for FSK decoding
    for (size_t c = 0; c < audio.count; c++) {
        // Convert float sample to fixed-point
        const int32_t sample_int = audio.p[c] * 32768.0f;
        int32_t current_sample = __SSAT(sample_int, 16);
        
        // Scale down to prevent overflow in multiplication
        current_sample /= 128;

        // Delay line for correlation detection
        delay_line[delay_line_index & 0x3F] = current_sample;

        // Correlate with delayed sample (similar to PDW's FSK demodulation)
        sample_mixed = (delay_line[(delay_line_index - 32) & 0x3F] * current_sample) / 4;
        sample_filtered = prev_mixed + sample_mixed + (prev_filtered / 2);

        delay_line_index++;

        prev_filtered = sample_filtered;
        prev_mixed = sample_mixed;

        // Slice the signal to get symbols
        sample_bits = (sample_bits << 1) | (sample_filtered < -20 ? 1 : 0);
        
        // Process the symbol through FLEX decoder state machine
        process_flex_symbol(sample_filtered < -20 ? 3 : 0);
    }
    
    // Send status updates every 250ms (approximated at our processing rate)
    sync_counter++;
    if (sync_counter >= 375) { // ~250ms at 1500Hz
        sync_counter = 0;
        send_status_update();
    }
}

void FlexProcessor::process_flex_symbol(uint8_t symbol) {
    // Shift sync buffer for sync pattern detection
    for (int i = 0; i < 3; i++) {
        sync_buffer[i] = sync_buffer[i + 1];
    }
    
    // Add new bit to sync buffer
    sync_buffer[3] = (sync_buffer[3] << 1) | (symbol > 0 ? 1 : 0);
    
    // If not synchronized, look for sync pattern
    if (!sync_found) {
        if (detect_flex_sync()) {
            sync_found = true;
            batch_index = 0;
            for (size_t i = 0; i < batch.size(); i++) {
                batch[i] = 0;
            }
            // Reset bit counter for frame information word
            bit_counter = 0;
            send_status_update();
        }
        return;
    }
    
    // We are synchronized, collect bits to form words
    phase += phase_inc;
    
    // Process bit at the right phase
    if (phase >= 0x10000) {
        phase &= 0xFFFF;
        
        // Add bit to current word
        uint32_t bit_value = (symbol > 0) ? 1 : 0;
        bit_counter++;
        bits_processed++;
        
        // In 4-level FSK, we need to handle 2 bits per symbol
        if (fsk_levels == 4) {
            // TODO: Implement 4-level FSK decoding
            // For now, just use 2-level FSK handling
        }
        
        // Add this bit to the current batch word
        if (batch_index < batch.size()) {
            batch[batch_index] = (batch[batch_index] << 1) | bit_value;
            
            // Each word is 32 bits
            if (bit_counter % 32 == 0) {
                // Process completed word
                batch_index++;
                
                // Process frame information from Block Information Word (BIW)
                if (batch_index == 1) {
                    decode_frame_info();
                }
                
                // Handle end of frame
                if (batch_index >= batch.size()) {
                    frames_processed++;
                    send_packet();
                    sync_found = false;
                }
            }
        }
    }
}

bool FlexProcessor::detect_flex_sync() {
    // First check if the central sync patterns match
    int center_errors = count_bits(sync_buffer[1] ^ SYNC1) + count_bits(sync_buffer[2] ^ SYNC2);
    
    // Allow a few bit errors (up to 4) in the central sync pattern
    if (center_errors <= 4) {
        // Check for various FLEX frame sync patterns
        for (int speed = 0; speed < 8; speed++) {
            int edge_errors = count_bits(sync_buffer[0] ^ flex_syncs[speed]) + 
                             count_bits(sync_buffer[3] ^ (flex_syncs[speed] ^ 0xFFFF));
            
            if (edge_errors <= 4) {  // Allow up to 4 bit errors
                // Found a valid sync pattern
                // Determine FLEX mode from sync pattern
                if ((speed & 0x03) == 0) {
                    baud_rate = FLEX_BAUD_1600;
                    phase_inc = 0x10000 * 1600 / audio_fs;
                } else if ((speed & 0x03) == 0x03) {
                    baud_rate = FLEX_BAUD_6400;
                    phase_inc = 0x10000 * 3200 / audio_fs;  // For 6400, we sample at 3200
                } else {
                    baud_rate = FLEX_BAUD_3200;
                    phase_inc = 0x10000 * 3200 / audio_fs;
                }
                
                fsk_levels = (speed & 0x02) ? 4 : 2;  // 2 or 4 level FSK
                phase = 0;
                
                // Save current timestamp for the packet
                timestamp = Timestamp::now();
                
                return true;
            }
        }
    }
    
    // Check for end-of-transmission pattern
    if ((sync_buffer[2] == EOT1) && (sync_buffer[3] == EOT2)) {
        sync_found = false;
        send_status_update();
    }
    
    return false;
}

void FlexProcessor::decode_frame_info() {
    // Apply BCH error correction to Block Information Word
    uint32_t biw = bch_correct(batch[0]);
    
    // Extract frame information
    current_cycle = (biw >> 19) & 0x0F;  // 4 bits
    current_frame = (biw >> 16) & 0x07;  // 3 bits
    
    // Send status update with new frame information
    send_status_update();
}

void FlexProcessor::process_addresses_and_vectors() {
    // Process the batch to correct errors in all words
    process_bch_block(batch.data(), batch.size());
    
    // Extract frame information from BIW (first word)
    uint32_t frame_info = batch[0];
    size_t vector_start = (frame_info >> 10) & 0x3F;     // Vector field start (6 bits)
    size_t address_start = ((frame_info >> 8) & 0x03) + 1;  // Address field start (2 bits)
    
    // Skip processing if vector start is at the same position as address start
    // (indicates an empty frame)
    if (vector_start == address_start) {
        return;
    }
    
    // Process all addresses and their associated vectors
    for (size_t addr_pos = address_start; addr_pos < vector_start && addr_pos < batch.size(); addr_pos++) {
        uint32_t address_word = batch[addr_pos];
        size_t vector_pos = vector_start + addr_pos - address_start;
        
        if (vector_pos >= batch.size()) continue;
        
        uint32_t vector_word = batch[vector_pos];
        
        // Check if this is a long address
        bool long_address = false;
        uint32_t address_data = address_word >> 11;
        
        if ((address_data & 0x7FFFF) < 0x008001 || 
            ((address_data & 0x7FFFF) > 0x1E0000 && (address_data & 0x7FFFF) < 0x1F0001) || 
            (address_data & 0x7FFFF) > 0x1F7FFE) {
            long_address = true;
        }
        
        // Skip if we have a long address but don't have the next word
        if (long_address && addr_pos + 1 >= batch.size()) {
            continue;
        }
        
        // Get message type from vector word
        uint8_t msg_type = (vector_word >> 4) & 0x07;
        
        // Process each message type differently
        switch (msg_type) {
            case TYPE_ALPHANUMERIC:
                // Process alphanumeric message
                // This would extract the message data from the batch words
                break;
                
            case TYPE_STANDARD_NUM:
            case TYPE_SPECIAL_NUM:
            case TYPE_NUMBERED_NUM:
                // Process numeric message
                break;
                
            case TYPE_TONE:
                // Process tone-only message
                break;
                
            case TYPE_SHORT_INSTR:
                // Process short instruction
                break;
                
            case TYPE_BINARY:
                // Process binary data
                break;
                
            case TYPE_SECURE:
                // Process secure message
                break;
        }
        
        // Skip next word if this was a long address
        if (long_address) {
            addr_pos++;
        }
    }
}

void FlexProcessor::process_bch_block(uint32_t* block, size_t count) {
    // Apply BCH error correction to each word in the block
    for (size_t i = 0; i < count; i++) {
        block[i] = bch_correct(block[i]);
    }
}

uint32_t FlexProcessor::bch_check(uint32_t codeword) {
    // Check the codeword using BCH error detection
    // Returns syndrome, which will be 0 if no errors
    uint32_t data = codeword >> 11;    // Upper 21 bits
    uint32_t check = codeword & 0x7FF; // Lower 11 bits
    
    uint32_t syndrome = 0;
    uint32_t temp = data;
    
    // Calculate syndrome
    for (int i = 0; i < 21; i++) {
        if (temp & 1) {
            syndrome ^= (0x7FF >> i);
        }
        temp >>= 1;
    }
    
    return syndrome ^ check;
}

uint32_t FlexProcessor::bch_correct(uint32_t codeword) {
    // Perform BCH error correction on the codeword
    // Returns corrected codeword, or original if uncorrectable
    
    uint32_t syndrome = bch_check(codeword);
    
    if (syndrome == 0) {
        // No errors detected
        return codeword;
    }
    
    // Simple single-bit error correction
    // For a full BCH implementation, a more complex algorithm is needed
    for (int i = 0; i < 32; i++) {
        uint32_t bit_pattern = static_cast<uint32_t>(1) << i;
        if (syndrome == bit_pattern) {
            // Single bit error at position i
            return codeword ^ bit_pattern;
        }
    }
    
    // Error detected but not correctable
    // Mark the word as bad by setting a high bit
    return codeword | 0x80000000;
}

int FlexProcessor::count_bits(uint32_t value) {
    // Count the number of set bits in a value
    int count = 0;
    while (value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

uint32_t FlexProcessor::reverse_bits(uint32_t value, int num_bits) {
    // Reverse the bits in a value
    uint32_t result = 0;
    for (int i = 0; i < num_bits; i++) {
        result = (result << 1) | (value & 1);
        value >>= 1;
    }
    return result;
}

void FlexProcessor::send_status_update() {
    // Send status message to the UI
    FlexStatsMessage message{bits_processed, frames_processed, sync_found, baud_rate};
    shared_memory.application_queue.push(message);
}

void FlexProcessor::send_packet() {
    // Create and send a FLEX packet to the UI
    FlexPacket packet;
    packet.set(batch);
    packet.set_timestamp(timestamp);
    packet.set_bitrate(baud_rate);
    
    // Set flags based on status
    packet.set_flag(FLEX_NORMAL);
    
    // Send packet message
    FlexPacketMessage message{packet};
    shared_memory.application_queue.push(message);
}

void FlexProcessor::reset() {
    // Reset the FLEX decoder state
    sync_found = false;
    batch_index = 0;
    bit_counter = 0;
    bits_processed = 0;
    frames_processed = 0;
    phase = 0;
    
    // Clear batch buffer
    for (size_t i = 0; i < batch.size(); i++) {
        batch[i] = 0;
    }
    
    // Clear sync buffer
    for (int i = 0; i < 4; i++) {
        sync_buffer[i] = 0;
    }
    
    // Reset delay line
    delay_line_index = 0;
    prev_mixed = 0;
    prev_filtered = 0;
    
    // Send status update
    send_status_update();
}

void FlexProcessor::on_message(const Message* const message) {
    if (message->id == Message::ID::FlexConfigure)
        configure(*reinterpret_cast<const FlexConfigureMessage*>(message));
}

void FlexProcessor::configure(const FlexConfigureMessage& message) {
    (void)message; // Prevent unused parameter warning
    
    // Configure the DSP chain for NFM with appropriate bandwidth for FLEX
    // Using 16kHz filter which is better for FLEX pager signals
    decim_0.configure(taps_16k0_decim_0.taps);
    decim_1.configure(taps_16k0_decim_1.taps);
    channel_filter.configure(taps_16k0_channel.taps, 2);
    
    // Configure FM demodulator with appropriate deviation for FLEX
    // FLEX typically uses +/-4.8kHz deviation
    demod.configure(audio_fs, 5000);

    // Configure audio output with appropriate filtering
    audio_output.configure(audio_24k_hpf_300hz_config, audio_24k_deemph_300_6_config, 0);
    
    // Reset the decoder state
    reset();
    
    configured = true;
    
    // Send initial status update
    send_status_update();
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<FlexProcessor>()};
    event_dispatcher.run();
    return 0;
}