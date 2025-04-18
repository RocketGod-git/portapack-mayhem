#include "ui_flex_rx.hpp"
#include "audio.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "portapack.hpp"
#include "portapack_shared_memory.hpp"
#include "rtc.h"

namespace ui::external_app::flex_rx {

void FlexRxView::focus() {
    field_frequency.focus();
}

FlexRxView::FlexRxView(NavigationView& nav) : nav_(nav) {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi, &channel, &field_rf_amp, &field_lna, &field_vga,
                  &field_volume, &field_frequency, &check_log, &text_debug,
                  &console, &sync_status});

    field_frequency.set_value(929614000);
    field_frequency.set_step(100);

    logging = true;
    check_log.set_value(logging);
    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
        if (logger) {
            rtc::RTC ts;
            rtcGetTime(&RTCD1, &ts);
            logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM Logging " + std::string(v ? "enabled" : "disabled"));
            console.writeln("[" + to_string_timestamp(ts) + "] Logging " + std::string(v ? "enabled" : "disabled"));
        }
    };

    logger = std::make_unique<FlexLogger>();
    if (logger) {
        auto result = logger->append(logs_dir / u"FLEX.TXT");
        rtc::RTC ts;
        rtcGetTime(&RTCD1, &ts);
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM FLEX Decoder started");
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM Logger initialized: " + (result.is_valid() ? "SUCCESS" : "FAILED - " + result.value().what()));
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM Recommended settings: NFM 16kHz BW, 929-932MHz");
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM Initial frequency: " + to_string_dec_uint(field_frequency.value()) + " Hz");
    }

    baseband::set_flex();
    audio::set_rate(audio::Rate::Hz_24000);
    audio::output::start();
    portapack::receiver_model.enable();

    console.writeln("FLEX Pager Decoder");
    console.writeln("NFM 16kHz BW, 929-932MHz");
    if (logger && logging) {
        rtc::RTC ts;
        rtcGetTime(&RTCD1, &ts);
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM Receiver initialized");
    }

    text_debug.set("BR:0 Sync:N Fr:0 Bits:0");
}

FlexRxView::~FlexRxView() {
    if (logger && logging) {
        rtc::RTC ts;
        rtcGetTime(&RTCD1, &ts);
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM FLEX Decoder shutting down");
    }
    audio::output::stop();
    portapack::receiver_model.disable();
    baseband::shutdown();
}

void FlexRxView::on_packet(const FlexPacketMessage& message) {
    const auto& packet = message.packet;
    const auto& batch = packet.batch();
    auto bitrate = packet.bitrate();
    rtc::RTC ts;
    rtcGetTime(&RTCD1, &ts);
    std::string str_log = "[" + to_string_timestamp(ts) + "] FLEX " + to_string_dec_uint(bitrate) + "bps ";

    if (logger && logging) {
        logger->log_decoded(ts, str_log + "Frame received - Baud: " + to_string_dec_uint(bitrate) + " Flag: " + to_string_dec_uint(packet.flag()) + " Batch size: " + to_string_dec_uint(batch.size()));
    }

    if (batch.size() < 8) {
        console.writeln("[" + to_string_timestamp(ts) + "] ERROR: Batch too small");
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "ERROR: Batch too small, size=" + to_string_dec_uint(batch.size()));
        }
        return;
    }

    uint32_t frame_info = batch[0];
    size_t vsa = (frame_info >> 10) & 0x3F;
    size_t asa = ((frame_info >> 8) & 0x03) + 1;
    uint8_t frame_id = (frame_info >> 16) & 0x7;
    uint8_t cycle_id = (frame_info >> 19) & 0xF;

    if (logger && logging) {
        logger->log_decoded(ts, str_log + "BIW: 0x" + to_string_hex(frame_info, 8) + " Frame: " + to_string_dec_uint(frame_id) + " Cycle: " + to_string_dec_uint(cycle_id) + " Addr start: " + to_string_dec_uint(asa) + " Vec start: " + to_string_dec_uint(vsa));
    }

    if (vsa <= asa) {
        console.writeln("[" + to_string_timestamp(ts) + "] ERROR: Invalid frame");
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "ERROR: Invalid frame structure, VSA=" + to_string_dec_uint(vsa) + " ASA=" + to_string_dec_uint(asa));
        }
        return;
    }

    for (size_t j = asa; j < vsa && j < batch.size(); j++) {
        uint32_t address_word = batch[j];
        if (address_word & 0x400000) {
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Address word " + to_string_dec_uint(j) + " marked bad: 0x" + to_string_hex(address_word, 8));
            }
            continue;
        }

        uint32_t data = address_word & 0x1FFFFF;
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "Address word " + to_string_dec_uint(j) + ": 0x" + to_string_hex(address_word, 8) + " Data: 0x" + to_string_hex(data, 5));
        }

        bool long_address = (data < 0x008001) || (data > 0x1E0000 && data < 0x1F0001) || (data > 0x1F7FFE);
        size_t vb = vsa + j - asa;
        if (vb >= batch.size()) {
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "ERROR: Vector word index out of bounds: " + to_string_dec_uint(vb));
            }
            continue;
        }

        uint32_t vector_word = batch[vb];
        if (vector_word & 0x400000) {
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Vector word " + to_string_dec_uint(vb) + " marked bad: 0x" + to_string_hex(vector_word, 8));
            }
            continue;
        }

        uint8_t vector_type = (vector_word >> 4) & 0x07;
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "Vector word " + to_string_dec_uint(vb) + ": 0x" + to_string_hex(vector_word, 8) + " Type: " + to_string_dec_uint(vector_type));
        }

        if (long_address && j + 1 >= batch.size()) {
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "ERROR: Long address missing second word");
            }
            continue;
        }

        uint32_t address;
        std::string display_text;
        if (!long_address) {
            address = data - 32768;
            display_text = to_string_dec_uint(address, 7);
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Short address: " + display_text);
            }
        } else {
            uint32_t second_word = batch[j + 1];
            if (second_word & 0x400000) {
                if (logger && logging) {
                    logger->log_decoded(ts, str_log + "Long address second word marked bad: 0x" + to_string_hex(second_word, 8));
                }
                continue;
            }
            uint32_t second_data = second_word & 0x1FFFFF;
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Long address second word: 0x" + to_string_hex(second_word, 8) + " Data: " + to_string_hex(second_data, 5));
            }
            address = ((second_data ^ 0x1FFFFF) << 15) + 2068480 + data;
            display_text = to_string_dec_uint(address, 9);
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Long address: " + display_text);
            }
            j++;
        }

        console.write("[" + to_string_timestamp(ts) + "] " + display_text);

        if (vector_type == 5) {
            int w1 = (vector_word >> 7) & 0x7F;
            int w2 = ((vector_word >> 14) & 0x7F) + w1 - 1;
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Alpha message: Words " + to_string_dec_uint(w1) + "-" + to_string_dec_uint(w2));
            }

            if (w1 < 0 || w2 < 0 || static_cast<size_t>(w1) >= batch.size() || static_cast<size_t>(w2) >= batch.size()) {
                if (logger && logging) {
                    logger->log_decoded(ts, str_log + "ERROR: Invalid message word range");
                }
                continue;
            }

            std::string message_text;
            for (int k = w1; k <= w2; k++) {
                uint32_t msg_word = batch[k];
                if (msg_word & 0x400000) {
                    if (logger && logging) {
                        logger->log_decoded(ts, str_log + "Message word " + to_string_dec_uint(k) + " marked bad: 0x" + to_string_hex(msg_word, 8));
                    }
                    continue;
                }
                if (logger && logging) {
                    logger->log_decoded(ts, str_log + "Message word " + to_string_dec_uint(k) + ": 0x" + to_string_hex(msg_word, 8));
                }
                for (int bit_pos = 0; bit_pos < 21; bit_pos += 7) {
                    uint8_t ch = (msg_word >> bit_pos) & 0x7F;
                    if (ch >= 32 && ch < 127) {
                        message_text += (char)ch;
                    }
                }
            }

            if (!message_text.empty()) {
                console.writeln(" \"" + message_text + "\"");
                if (logger && logging) {
                    logger->log_decoded(ts, str_log + "Message: \"" + message_text + "\"");
                }
            }
        } else if (vector_type == 2) {
            console.writeln(" TONE ONLY");
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "Tone-only message");
            }
        }

        if (logger && logging) {
            logger->log_decoded(ts, str_log + "-----------------------------------");
        }
    }

    if (logger && logging) {
        logger->log_decoded(ts, str_log + "End of frame processing");
    }
}

void FlexRxView::on_stats(const FlexStatsMessage& stats) {
    rtc::RTC ts;
    rtcGetTime(&RTCD1, &ts);
    std::string str_log = "[" + to_string_timestamp(ts) + "] STATS ";

    if (stats.has_sync != last_has_sync) {
        std::string sync_log = stats.has_sync ? "SYNC ACQUIRED" : "SYNC LOST";
        if (stats.has_sync) {
            sync_log += " Baud: " + to_string_dec_uint(stats.baud_rate);
        } else {
            sync_log += " No valid sync detected";
            if (logger && logging) {
                logger->log_decoded(ts, str_log + "ERROR: " + sync_log);
            }
        }
        console.writeln("[" + to_string_timestamp(ts) + "] " + sync_log);
        sync_status.set_color(stats.has_sync ? Color::green() : Color::red());
        if (logger && logging) {
            logger->log_decoded(ts, str_log + sync_log);
        }
    }

    if (stats.baud_rate != last_baud_rate && stats.baud_rate != 0) {
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "Baud rate: " + to_string_dec_uint(stats.baud_rate));
        }
    }

    if (stats.current_frames != last_frames) {
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "Frame counter: " + to_string_dec_uint(stats.current_frames));
        }
    }

    if ((stats.current_bits / 1000) != (last_bits / 1000)) {
        if (logger && logging) {
            logger->log_decoded(ts, str_log + "Bit counter: " + to_string_dec_uint(stats.current_bits));
        }
    }

    if (stats.baud_rate != last_baud_rate ||
        stats.has_sync != last_has_sync ||
        stats.current_frames != last_frames ||
        (stats.current_bits / 100) != (last_bits / 100)) {
        std::string debug_text = "BR:" + to_string_dec_uint(stats.baud_rate) +
                               " Sync:" + (stats.has_sync ? "Y" : "N") +
                               " Fr:" + to_string_dec_uint(stats.current_frames) +
                               " Bits:" + to_string_dec_uint(stats.current_bits);
        text_debug.set(debug_text);
        if (logger && logging && (stats.current_bits / 1000) != (last_bits / 1000)) {
            logger->log_decoded(ts, str_log + "Status: " + debug_text);
        }
    }

    last_baud_rate = stats.baud_rate;
    last_has_sync = stats.has_sync;
    last_frames = stats.current_frames;
    last_bits = stats.current_bits;
}

void FlexRxView::on_freqchg(int64_t freq) {
    rtc::RTC ts;
    rtcGetTime(&RTCD1, &ts);
    if (logger && logging) {
        logger->log_decoded(ts, "[" + to_string_timestamp(ts) + "] SYSTEM Frequency changed to " + to_string_dec_uint(freq) + " Hz");
        console.writeln("[" + to_string_timestamp(ts) + "] Freq: " + to_string_dec_uint(freq) + " Hz");
    }
    field_frequency.set_value(freq);
}

} // namespace ui::external_app::flex_rx