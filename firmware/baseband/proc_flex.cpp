#include "proc_flex.hpp"

#include "event_m4.hpp"
#include "audio_dma.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

using namespace std;

namespace {
uint8_t diff_bit_count(uint32_t left, uint32_t right) {
    uint32_t diff = left ^ right;
    uint8_t count = 0;
    for (size_t i = 0; i < sizeof(diff) * 8; ++i) {
        if (((diff >> i) & 0x1) == 1)
            ++count;
    }
    return count;
}

uint32_t bch_correct(uint32_t data) {
    uint32_t syndrome = 0;
    uint32_t corrected = data;
    for (size_t i = 0; i < 21; ++i) {
        if ((data >> i) & 0x1)
            syndrome ^= (1 << (i % 10));
    }
    if (syndrome) {
        for (size_t i = 0; i < 21; ++i) {
            if ((syndrome >> (i % 10)) & 0x1) {
                corrected ^= (1 << i);
                syndrome = 0;
                for (size_t j = 0; j < 21; ++j) {
                    if ((corrected >> j) & 0x1)
                        syndrome ^= (1 << (j % 10));
                }
                if (syndrome == 0)
                    return corrected;
                corrected = data;
            }
        }
        return data | 0x400000;
    }
    return data;
}
}

void AudioNormalizer::execute_in_place(const buffer_f32_t& audio) {
    if (counter_ >= 24000) {
        max_ *= 0.9f;
        min_ *= 0.9f;
        counter_ = 0;
        calculate_thresholds();
    }

    counter_ += audio.count;

    for (size_t i = 0; i < audio.count; ++i) {
        auto& val = audio.p[i];

        if (val > max_) {
            max_ = val;
            calculate_thresholds();
        }
        if (val < min_) {
            min_ = val;
            calculate_thresholds();
        }

        if (val >= t_hi_)
            val = 1.0f;
        else if (val <= t_lo_)
            val = -1.0f;
        else
            val = 0.0f;
    }
}

void AudioNormalizer::calculate_thresholds() {
    auto center = (max_ + min_) / 2.0f;
    auto range = (max_ - min_) / 2.0f;
    auto threshold = range * 0.1;
    t_hi_ = center + threshold;
    t_lo_ = center - threshold;
}

void BitQueue::push(bool bit) {
    data_ = (data_ << 1) | (bit ? 1 : 0);
    if (count_ < max_size_) ++count_;
}

bool BitQueue::pop() {
    if (count_ == 0) return false;
    --count_;
    return (data_ & (1 << count_)) != 0;
}

void BitQueue::reset() {
    data_ = 0;
    count_ = 0;
}

uint8_t BitQueue::size() const {
    return count_;
}

uint32_t BitQueue::data() const {
    return data_;
}

void BitExtractor::extract_bits(const buffer_f32_t& audio) {
    for (size_t i = 0; i < audio.count; ++i) {
        auto sample = audio.p[i];

        if (current_rate_) {
            if (current_rate_->handle_sample(sample)) {
                auto value = (current_rate_->bits.data() & 1) == 1;
                bits_.push(value);
            }
        } else {
            for (auto& rate : known_rates_) {
                if (rate.handle_sample(sample) &&
                    diff_bit_count(rate.bits.data(), clock_magic_number) <= 3) {
                    rate.is_stable = true;
                    current_rate_ = &rate;
                }
            }
        }
    }
}

void BitExtractor::configure(uint32_t sample_rate) {
    sample_rate_ = sample_rate;
    for (auto& rate : known_rates_)
        rate.sample_interval = sample_rate / (2.0 * rate.baud_rate);
}

void BitExtractor::reset() {
    current_rate_ = nullptr;
    for (auto& rate : known_rates_)
        rate.reset();
}

uint16_t BitExtractor::baud_rate() const {
    return current_rate_ ? current_rate_->baud_rate : 0;
}

bool BitExtractor::RateInfo::handle_sample(float sample) {
    samples_until_next -= 1;

    if (samples_until_next > 0)
        return false;

    bool value = signbit(sample);
    bool bit_pushed = false;

    switch (state) {
        case State::WaitForSample:
            state = State::ReadyToSend;
            break;

        case State::ReadyToSend:
            if (!is_stable && prev_value != value) {
                samples_until_next += (sample_interval / 8.0);
            } else {
                state = State::WaitForSample;
                bit_pushed = true;
                bits.push(value);
            }
            break;
    }

    samples_until_next += sample_interval;
    prev_value = value;

    return bit_pushed;
}

void BitExtractor::RateInfo::reset() {
    state = State::WaitForSample;
    samples_until_next = 0.0;
    prev_value = false;
    is_stable = false;
    bits.reset();
}

void FrameExtractor::process_bits() {
    while (bits_.size() > 0) {
        take_one_bit();

        if (bit_count_ < data_bit_count)
            continue;

        if (!has_sync_) {
            if (diff_bit_count(data_, sync_codeword) <= 2) {
                handle_sync(false);
            } else if (diff_bit_count(data_, ~sync_codeword) <= 2) {
                handle_sync(true);
            } else if (diff_bit_count(data_, eot_codeword) <= 2) {
                reset();
                return;
            } else {
                sync_fail_count_++;
                clear_data_bits();
                continue;
            }
        }

        if (block_bit_index_ < 256) {
            block_buffer_[block_bit_index_++] = data_ & 0x1;
            if (block_bit_index_ == 256)
                process_block();
            clear_data_bits();
            continue;
        }

        save_current_word();

        if (word_count_ == flex_batch_size)
            handle_frame_complete();
    }
}

void FrameExtractor::flush() {
    if (word_count_ == 0) return;
    handle_frame_complete();
}

void FrameExtractor::reset() {
    clear_data_bits();
    has_sync_ = false;
    inverted_ = false;
    word_count_ = 0;
    block_count_ = 0;
    block_bit_index_ = 0;
    sync_fail_count_ = 0;
}

void FrameExtractor::clear_data_bits() {
    data_ = 0;
    bit_count_ = 0;
}

void FrameExtractor::take_one_bit() {
    data_ = (data_ << 1) | bits_.pop();
    if (bit_count_ < data_bit_count)
        ++bit_count_;
}

void FrameExtractor::handle_sync(bool inverted) {
    clear_data_bits();
    has_sync_ = true;
    inverted_ = inverted;
    word_count_ = 0;
    block_count_ = 0;
    block_bit_index_ = 0;
    sync_fail_count_ = 0;
}

void FrameExtractor::save_current_word() {
    batch_[word_count_++] = inverted_ ? ~data_ : bch_correct(data_);
    clear_data_bits();
}

void FrameExtractor::handle_frame_complete() {
    on_batch_(*this);
    has_sync_ = false;
    word_count_ = 0;
    block_count_ = 0;
    block_bit_index_ = 0;
    sync_fail_count_ = 0;
}

void FrameExtractor::process_block() {
    for (uint8_t i = 0; i < 8; ++i) {
        decode_word(block_count_, i);
    }
    ++block_count_;
    block_bit_index_ = 0;
}

void FrameExtractor::decode_word(uint8_t block_index, uint8_t word_index) {
    uint32_t word = 0;
    for (uint8_t j = 0; j < 32; ++j) {
        uint8_t bit_index = (j * 8) + word_index;
        word = (word << 1) | (block_buffer_[bit_index] & 0x1);
    }
    batch_[block_index * 8 + word_index] = inverted_ ? ~word : bch_correct(word);
}

void FlexProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured) return;

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);
    auto audio = demod.execute(channel_out, audio_buffer);

    bool has_audio = squelch.execute(audio);
    squelch_history = (squelch_history << 1) | (has_audio ? 1 : 0);

    if (squelch_history == 0) {
        if (frame_extractor.current() > 0) {
            flush();
            reset();
            send_stats();
        }

        for (size_t i = 0; i < audio.count; ++i)
            audio.p[i] = 0.0;

        audio_output.write(audio);
        return;
    }

    normalizer.execute_in_place(audio);
    audio_output.write(audio);

    bit_extractor.extract_bits(audio);
    frame_extractor.process_bits();

    samples_processed += buffer.count;
    sync_samples_processed += buffer.count;

    if (sync_samples_processed >= sync_timeout_threshold && frame_extractor.has_sync()) {
        if (frame_extractor.count() == 0) {
            reset();
            send_stats();
        }
        sync_samples_processed = 0;
    }

    if (samples_processed >= stat_update_threshold) {
        send_stats();
        samples_processed -= stat_update_threshold;
    }
}

void FlexProcessor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::FlexConfigure:
            configure();
            break;

        case Message::ID::NBFMConfigure: {
            auto config = reinterpret_cast<const NBFMConfigureMessage*>(message);
            squelch.set_threshold(config->squelch_level / 99.0);
            break;
        }

        case Message::ID::AudioBeep:
            on_beep_message(*reinterpret_cast<const AudioBeepMessage*>(message));
            break;

        default:
            break;
    }
}

void FlexProcessor::configure() {
    constexpr size_t decim_0_output_fs = baseband_fs / decim_0.decimation_factor;
    constexpr size_t decim_1_output_fs = decim_0_output_fs / decim_1.decimation_factor;
    constexpr size_t channel_filter_output_fs = decim_1_output_fs / 2;
    constexpr size_t demod_input_fs = channel_filter_output_fs;

    decim_0.configure(taps_16k0_decim_0.taps);
    decim_1.configure(taps_16k0_decim_1.taps);
    channel_filter.configure(taps_16k0_channel.taps, 2);
    demod.configure(demod_input_fs, 4500);

    audio_output.configure(false);

    bit_extractor.configure(demod_input_fs);
    squelch.set_threshold(0.5); // Default threshold, adjustable via NBFMConfigure

    configured = true;
}

void FlexProcessor::flush() {
    frame_extractor.flush();
}

void FlexProcessor::reset() {
    bits.reset();
    bit_extractor.reset();
    frame_extractor.reset();
    samples_processed = 0;
    sync_samples_processed = 0;
    squelch_history = 0;
}

void FlexProcessor::send_stats() const {
    FlexStatsMessage message(
        frame_extractor.current(), frame_extractor.count(),
        frame_extractor.has_sync(), bit_extractor.baud_rate());
    shared_memory.application_queue.push(message);
}

void FlexProcessor::send_packet() {
    packet.set_flag(FlexPacketFlag::FLEX_NORMAL);
    packet.set_timestamp(Timestamp::now());
    packet.set_bitrate(bit_extractor.baud_rate());
    packet.set(frame_extractor.batch());

    FlexPacketMessage message(packet);
    shared_memory.application_queue.push(message);

    sync_samples_processed = 0; // Reset sync timeout on valid packet
}

void FlexProcessor::on_beep_message(const AudioBeepMessage& message) {
    audio::dma::beep_start(message.freq, message.sample_rate, message.duration_ms);
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<FlexProcessor>()};
    event_dispatcher.run();
    return 0;
}