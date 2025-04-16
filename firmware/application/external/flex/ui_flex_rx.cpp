/*
 * ------------------------------------------------------------
 * |  Made by RocketGod                                       |
 * |  Find me at https://betaskynet.com                       |
 * |  Argh matey!                                             |
 * ------------------------------------------------------------
 */

#include "ui_flex_rx.hpp"
#include "audio.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "portapack.hpp"
#include "portapack_shared_memory.hpp"

namespace ui::external_app::flex_rx {

void FlexRxView::focus() {
    field_frequency.focus();
}

FlexRxView::FlexRxView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi, &channel, &field_rf_amp, &field_lna, &field_vga,
                  &field_volume, &field_frequency, &check_log, &text_debug,
                  &console, &sync_status});

    // Set default frequency to common FLEX pager frequency
    field_frequency.set_value(929614000);
    field_frequency.set_step(100);

    // Enable logging by default
    logging = true;
    check_log.set_value(logging);
    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
        
        // Log logging state change
        if (logger) {
            Timestamp ts;
            std::string log_status = "SYSTEM Logging " + std::string(v ? "enabled" : "disabled");
            logger->log_decoded(ts, log_status);
            
            // Also show on console
            console.writeln(log_status);
        }
    };

    // Initialize logger
    logger = std::make_unique<FlexLogger>();
    if (logger) {
        auto result = logger->append(logs_dir / u"FLEX.TXT");
        
        // Log application startup
        Timestamp ts;
        logger->log_decoded(ts, "SYSTEM FLEX Decoder started");
        
        // Log logger initialization status
        std::string logger_status = "SYSTEM Logger initialized: ";
        if (result.is_valid()) {
            logger_status += "SUCCESS";
        } else {
            logger_status += "FAILED - " + result.value().what();
        }
        logger->log_decoded(ts, logger_status);
        
        // Log recommended settings
        logger->log_decoded(ts, "SYSTEM Recommended settings: NFM 16kHz BW, 929-932MHz");
        
        // Log initial frequency
        logger->log_decoded(ts, "SYSTEM Initial frequency: " + 
                          to_string_dec_uint(field_frequency.value()) + " Hz");
    }

    baseband::set_flex();

    // Log audio initialization
    if (logger && logging) {
        Timestamp ts;
        logger->log_decoded(ts, "SYSTEM Audio initialized at 24kHz");
    }
    
    audio::set_rate(audio::Rate::Hz_24000);
    audio::output::start();

    portapack::receiver_model.enable();
    
    // Display setup instructions
    console.writeln("FLEX Pager Decoder");
    console.writeln("Use NFM 16kHz BW, 929-932MHz");
    
    // Log receiver initialization
    if (logger && logging) {
        Timestamp ts;
        logger->log_decoded(ts, "SYSTEM Receiver initialized");
    }
    
    // Force an initial debug info update
    text_debug.set("BR:0 Sync:N Fr:0 Bits:0");
}

FlexRxView::~FlexRxView() {
    // Log application shutdown
    if (logger && logging) {
        Timestamp ts;
        logger->log_decoded(ts, "SYSTEM FLEX Decoder shutting down");
    }
    
    audio::output::stop();
    portapack::receiver_model.disable();
    baseband::shutdown();
}

void FlexRxView::on_manual_freq_change(int64_t freq) {
    // This is called when user manually changes frequency via the UI
    if (logger && logging) {
        Timestamp ts;
        std::string freq_log = "SYSTEM User changed frequency to " + 
                            to_string_dec_uint(freq) + " Hz";
        logger->log_decoded(ts, freq_log);
        
        // Also show on console
        console.writeln("Freq: " + to_string_dec_uint(freq) + " Hz");
    }
}

std::string FlexRxView::to_string_hex(uint32_t value, int width) {
    char buffer[16];
    if (width <= 2)
        snprintf(buffer, sizeof(buffer), "%02lX", value);
    else if (width <= 4)
        snprintf(buffer, sizeof(buffer), "%04lX", value);
    else if (width <= 6)
        snprintf(buffer, sizeof(buffer), "%06lX", value);
    else
        snprintf(buffer, sizeof(buffer), "%08lX", value);
    return std::string(buffer);
}

std::string FlexRxView::get_message_type_name(uint8_t type) {
    switch (type) {
        case 0: return "Secure";
        case 1: return "Short Instruction";
        case 2: return "Tone/Numeric";
        case 3: return "Standard Numeric";
        case 4: return "Special Numeric";
        case 5: return "Alphanumeric";
        case 6: return "Binary";
        case 7: return "Numbered Numeric";
        default: return "Unknown";
    }
}

uint32_t FlexRxView::check_bch_checksum(uint32_t codeword) {
    uint32_t data = codeword >> 11;
    uint32_t check = codeword & 0x7FF;
    
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

void FlexRxView::on_packet(const FlexPacketMessage& message) {
    const auto& packet = message.packet;
    const auto& batch = packet.batch();
    auto bitrate = packet.bitrate();
    std::string str_log = "FLEX " + to_string_dec_uint(bitrate) + "bps ";
    
    // Log packet received
    std::string frame_log = "Frame received - Baud: " + to_string_dec_uint(bitrate) + 
                          " Flag: " + to_string_dec_uint(packet.flag()) + 
                          " Batch size: " + to_string_dec_uint(batch.size());
    
    if (logger && logging) {
        Timestamp ts;
        logger->log_decoded(ts, frame_log);
    }

    // Log early exit conditions
    if (batch.size() < 8) {
        std::string error_log = "ERROR: Batch too small, size=" + to_string_dec_uint(batch.size());
        console.writeln(error_log);
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + error_log);
        }
        return;
    }

    // Extract and log frame information
    uint32_t frame_info = batch[0];
    size_t vsa = (frame_info >> 10) & 0x3F;
    size_t asa = ((frame_info >> 8) & 0x03) + 1;
    uint8_t frame_id = (frame_info >> 16) & 0x7;
    uint8_t cycle_id = (frame_info >> 19) & 0xF;
    
    std::string biw_log = "BIW: 0x" + to_string_hex(frame_info, 8) + 
                         " Frame: " + to_string_dec_uint(frame_id) + 
                         " Cycle: " + to_string_dec_uint(cycle_id) + 
                         " Addr start: " + to_string_dec_uint(asa) + 
                         " Vec start: " + to_string_dec_uint(vsa);
    
    if (logger && logging) {
        Timestamp ts;
        logger->log_decoded(ts, str_log + biw_log);
    }

    // Only process the frame if we have valid addresses
    if (vsa <= asa) {
        std::string error_log = "ERROR: Invalid frame structure, VSA=" + 
                              to_string_dec_uint(vsa) + " ASA=" + to_string_dec_uint(asa);
        console.writeln(error_log);
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + error_log);
        }
        return;
    }

    // Process each address
    for (size_t j = asa; j < vsa && j < batch.size(); j++) {
        uint32_t address_word = batch[j];
        
        // Skip words marked as bad (high bit set by error correction)
        if (address_word & 0x80000000) {
            std::string error_log = "Address word " + to_string_dec_uint(j) + 
                                  " marked bad: 0x" + to_string_hex(address_word, 8);
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + error_log);
            }
            continue;
        }
        
        // Extract address data
        uint32_t data = address_word >> 11;
        
        std::string addr_log = "Address word " + to_string_dec_uint(j) + 
                             ": 0x" + to_string_hex(address_word, 8) + 
                             " Data: 0x" + to_string_hex(data, 5);
        
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + addr_log);
        }

        // Check if this is a long address
        bool long_address = (data & 0x7FFFF) < 0x008001 || 
                          ((data & 0x7FFFF) > 0x1E0000 && (data & 0x7FFFF) < 0x1F0001) || 
                          (data & 0x7FFFF) > 0x1F7FFE;
        
        // Get corresponding vector word
        size_t vb = vsa + j - asa;
        if (vb >= batch.size()) {
            std::string error_log = "ERROR: Vector word index out of bounds: " + 
                                  to_string_dec_uint(vb) + " >= " + to_string_dec_uint(batch.size());
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + error_log);
            }
            continue;
        }

        uint32_t vector_word = batch[vb];
        
        // Skip words marked as bad
        if (vector_word & 0x80000000) {
            std::string error_log = "Vector word " + to_string_dec_uint(vb) + 
                                  " marked bad: 0x" + to_string_hex(vector_word, 8);
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + error_log);
            }
            continue;
        }
        
        // Extract vector information
        uint32_t vector_data = vector_word >> 11;
        uint8_t vector_type = (vector_data >> 4) & 0x07;
        
        std::string vec_log = "Vector word " + to_string_dec_uint(vb) + 
                            ": 0x" + to_string_hex(vector_word, 8) + 
                            " Data: 0x" + to_string_hex(vector_data, 5) + 
                            " Type: " + to_string_dec_uint(vector_type) + 
                            " (" + get_message_type_name(vector_type) + ")";
        
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + vec_log);
        }
        
        // Skip if long address but don't have second word
        if (long_address && j + 1 >= batch.size()) {
            std::string error_log = "ERROR: Long address but missing second word";
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + error_log);
            }
            continue;
        }

        // Calculate address
        std::string display_text;
        uint32_t address;
        
        if (!long_address) {
            address = (data & 0x7FFFF) - 32768;
            display_text = to_string_dec_uint(address);
            
            std::string addr_info = "Short address: " + display_text;
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + addr_info);
            }
        } else {
            uint32_t second_word = batch[j + 1];
            
            // Skip if second word is bad
            if (second_word & 0x80000000) {
                std::string error_log = "Long address second word marked bad: 0x" + 
                                      to_string_hex(second_word, 8);
                if (logger && logging) {
                    Timestamp ts;
                    logger->log_decoded(ts, str_log + error_log);
                }
                continue;
            }
            
            uint32_t second_data = second_word >> 11;
            
            std::string second_log = "Long address second word: 0x" + 
                                   to_string_hex(second_word, 8) + 
                                   " Data: 0x" + to_string_hex(second_data, 5);
            
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + second_log);
            }
            
            // Calculate long address according to FLEX protocol
            address = ((second_data ^ 0x1FFFFF) << 15) + 2068480 + (data & 0x7FFFF);
            display_text = to_string_dec_uint(address);
            
            std::string addr_info = "Long address: " + display_text;
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + addr_info);
            }
            
            j++; // Skip next word since we processed it as part of long address
        }

        // Display the address on console
        console.writeln(display_text);
        
        // Process different message types
        if (vector_type == 5) {  // TYPE_ALPHANUMERIC
            // Extract message location information
            int w1 = (vector_data >> 7) & 0x7F;
            int w2 = ((vector_data >> 14) & 0x7F) + w1 - 1;
            
            std::string msg_info = "Alpha message: Words " + to_string_dec_uint(w1) + 
                                 "-" + to_string_dec_uint(w2);
            
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + msg_info);
            }
            
            // Validate message word range
            if (w1 < 0 || w2 < 0 || static_cast<size_t>(w1) >= batch.size() || 
                static_cast<size_t>(w2) >= batch.size()) {
                std::string error_log = "ERROR: Invalid message word range";
                
                if (logger && logging) {
                    Timestamp ts;
                    logger->log_decoded(ts, str_log + error_log);
                }
                continue;
            }

            // Extract message text
            std::string message_text;
            for (int k = w1; k <= w2; k++) {
                uint32_t msg_word = batch[k];
                
                // Skip bad words
                if (msg_word & 0x80000000) {
                    std::string error_log = "Message word " + to_string_dec_uint(k) + 
                                          " marked bad: 0x" + to_string_hex(msg_word, 8);
                    
                    if (logger && logging) {
                        Timestamp ts;
                        logger->log_decoded(ts, str_log + error_log);
                    }
                    continue;
                }
                
                std::string word_log = "Message word " + to_string_dec_uint(k) + 
                                     ": 0x" + to_string_hex(msg_word, 8);
                
                if (logger && logging) {
                    Timestamp ts;
                    logger->log_decoded(ts, str_log + word_log);
                }
                
                // Extract characters (3 per word, 7 bits each)
                for (int bit_pos = 0; bit_pos < 21; bit_pos += 7) {
                    uint8_t ch = (msg_word >> bit_pos) & 0x7F;
                    if (ch >= 32 && ch < 127) {
                        message_text += (char)ch;
                    } else if (ch != 0 && ch != 0x03) {
                        // Log non-printable characters (except NULL and ETX)
                        std::string char_log = "Non-printable char: 0x" + 
                                             to_string_hex(ch, 2);
                        
                        if (logger && logging) {
                            Timestamp ts;
                            logger->log_decoded(ts, str_log + char_log);
                        }
                    }
                }
            }
            
            // Display and log the message
            if (!message_text.empty()) {
                console.writeln(message_text);
                
                std::string msg_log = "Message: \"" + message_text + "\"";
                if (logger && logging) {
                    Timestamp ts;
                    logger->log_decoded(ts, str_log + msg_log);
                }
            } else {
                std::string msg_log = "Empty message";
                if (logger && logging) {
                    Timestamp ts;
                    logger->log_decoded(ts, str_log + msg_log);
                }
            }
        } else if (vector_type == 3 || vector_type == 4 || vector_type == 7) {
            // Numeric message types
            std::string msg_log = "Numeric message type " + to_string_dec_uint(vector_type) + 
                                " - Not fully implemented";
            
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + msg_log);
            }
            
            // TODO: Implement numeric message decoding
        } else if (vector_type == 2) {
            // Tone-only message
            std::string msg_log = "Tone-only message";
            
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + msg_log);
            }
            
            console.writeln("TONE ONLY");
        } else {
            // Other message types
            std::string msg_log = "Message type " + to_string_dec_uint(vector_type) + 
                                " (" + get_message_type_name(vector_type) + ") - Not implemented";
            
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + msg_log);
            }
        }
        
        // Add separator in log for clarity
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + "-----------------------------------");
        }
    }
    
    // Log end of frame processing
    std::string end_log = "End of frame processing";
    if (logger && logging) {
        Timestamp ts;
        logger->log_decoded(ts, str_log + end_log);
    }
    
    console_color++;
}

void FlexRxView::on_stats(const FlexStatsMessage& stats) {
    // Create log prefix
    std::string str_log = "STATS ";
    
    // Log all status changes
    if (stats.has_sync != last_has_sync) {
        std::string sync_log = stats.has_sync ? "SYNC ACQUIRED" : "SYNC LOST";
        
        if (stats.has_sync) {
            sync_log += " - Baud: " + to_string_dec_uint(stats.baud_rate) + 
                      " Frames: " + to_string_dec_uint(stats.current_frames) + 
                      " Bits: " + to_string_dec_uint(stats.current_bits);
        }
        
        console.writeln(sync_log);
        sync_status.set_color(stats.has_sync ? Color::green() : Color::red());
        
        if (logger && logging) {
            // Create a timestamp for this event
            Timestamp ts;
            logger->log_decoded(ts, str_log + sync_log);
        }
    }

    // Log baud rate changes
    if (stats.baud_rate != last_baud_rate && last_baud_rate != 0) {
        std::string baud_log = "Baud rate changed: " + to_string_dec_uint(last_baud_rate) + 
                            " -> " + to_string_dec_uint(stats.baud_rate);
        
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + baud_log);
        }
    } else if (stats.baud_rate != 0 && last_baud_rate == 0) {
        // Initial baud rate setting
        std::string baud_log = "Baud rate changed: " + to_string_dec_uint(last_baud_rate) + 
                            " -> " + to_string_dec_uint(stats.baud_rate);
        
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + baud_log);
        }
    }
    
    // Log frame counter changes
    if (stats.current_frames != last_frames) {
        std::string frame_log = "Frame counter: " + to_string_dec_uint(stats.current_frames);
        
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + frame_log);
        }
    }
    
    // Log significant bit counter changes (every 1000 bits)
    if ((stats.current_bits / 1000) != (last_bits / 1000)) {
        std::string bit_log = "Bit counter: " + to_string_dec_uint(stats.current_bits);
        
        if (logger && logging) {
            Timestamp ts;
            logger->log_decoded(ts, str_log + bit_log);
        }
    }

    // Update the debug text display
    if (stats.baud_rate != last_baud_rate ||
        stats.has_sync != last_has_sync ||
        stats.current_frames != last_frames ||
        (stats.current_bits / 100) != (last_bits / 100)) {
        
        std::string debug_text = "BR:" + to_string_dec_uint(stats.baud_rate) +
                               " Sync:" + (stats.has_sync ? "Y" : "N") +
                               " Fr:" + to_string_dec_uint(stats.current_frames) +
                               " Bits:" + to_string_dec_uint(stats.current_bits);
        
        text_debug.set(debug_text);
        
        // Log detailed stats periodically
        if ((stats.current_bits / 1000) != (last_bits / 1000) || stats.baud_rate != last_baud_rate) {
            std::string detailed_log = "Status: " + debug_text;
            
            if (logger && logging) {
                Timestamp ts;
                logger->log_decoded(ts, str_log + detailed_log);
            }
        }
    }

    // Update last values
    last_baud_rate = stats.baud_rate;
    last_has_sync = stats.has_sync;
    last_frames = stats.current_frames;
    last_bits = stats.current_bits;
}

void FlexRxView::on_freqchg(int64_t freq) {
    // Log the change
    if (logger && logging) {
        Timestamp ts;
        std::string freq_log = "SYSTEM External frequency change to " + 
                            to_string_dec_uint(freq) + " Hz";
        logger->log_decoded(ts, freq_log);
        
        // Also display on console
        console.writeln("Ext Freq: " + to_string_dec_uint(freq) + " Hz");
    }
    
    // Update the frequency field
    field_frequency.set_value(freq);
}

}  // namespace ui::external_app::flex_rx