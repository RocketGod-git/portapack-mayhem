#ifndef __PROC_FLEX_H__
#define __PROC_FLEX_H__

#include "audio_output.hpp"
#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "dsp_decimate.hpp"
#include "dsp_demodulate.hpp"
#include "message.hpp"
#include "portapack_shared_memory.hpp"
#include "rssi_thread.hpp"

#include <array>
#include <cstdint>
#include <functional>

class AudioNormalizer {
public:
    void execute_in_place(const buffer_f32_t& audio);

private:
    void calculate_thresholds();

    uint32_t counter_ = 0;
    float min_ = 99.0f;
    float max_ = -99.0f;
    float t_hi_ = 1.0;
    float t_lo_ = 1.0;
};

class BitQueue {
public:
    void push(bool bit);
    bool pop();
    void reset();
    uint8_t size() const;
    uint32_t data() const;

private:
    uint32_t data_ = 0;
    uint8_t count_ = 0;

    static constexpr uint8_t max_size_ = sizeof(data_) * 8;
};

class BitExtractor {
public:
    BitExtractor(BitQueue& bits) : bits_{bits} {}

    void extract_bits(const buffer_f32_t& audio);
    void configure(uint32_t sample_rate);
    void reset();
    uint16_t baud_rate() const;

private:
    static constexpr uint32_t clock_magic_number = 0xA6C6AAAA;

    struct RateInfo {
        enum class State : uint8_t {
            WaitForSample,
            ReadyToSend
        };

        const int16_t baud_rate = 0;
        float sample_interval = 0.0;

        State state = State::WaitForSample;
        float samples_until_next = 0.0;
        bool prev_value = false;
        bool is_stable = false;
        BitQueue bits{};

        bool handle_sample(float sample);
        void reset();
    };

    std::array<RateInfo, 3> known_rates_{
        RateInfo{1600},
        RateInfo{3200},
        RateInfo{6400}};

    BitQueue& bits_;

    uint32_t sample_rate_ = 0;
    RateInfo* current_rate_ = nullptr;
};

class FrameExtractor {
public:
    using batch_t = flex_batch_t;
    using batch_handler_t = std::function<void(FrameExtractor&)>;

    FrameExtractor(BitQueue& bits, batch_handler_t on_batch)
        : bits_{bits}, on_batch_{on_batch} {}

    void process_bits();
    void flush();
    void reset();

    const batch_t& batch() const { return batch_; }
    uint32_t current() const { return data_; }
    uint8_t count() const { return word_count_; }
    bool has_sync() const { return has_sync_; }

private:
    static constexpr uint32_t sync_codeword = 0xA6C6AAAA;
    static constexpr uint32_t eot_codeword = 0xAAAAFFFF;
    static constexpr uint8_t data_bit_count = sizeof(uint32_t) * 8;

    void clear_data_bits();
    void take_one_bit();
    void handle_sync(bool inverted);
    void save_current_word();
    void handle_frame_complete();
    void process_block();
    void decode_word(uint8_t block_index, uint8_t word_index);

    BitQueue& bits_;
    batch_handler_t on_batch_{};

    bool has_sync_ = false;
    bool inverted_ = false;
    uint32_t data_ = 0;
    uint8_t bit_count_ = 0;
    uint8_t word_count_ = 0;
    uint8_t block_count_ = 0;
    batch_t batch_{};
    std::array<char, 256> block_buffer_{};
    uint16_t block_bit_index_ = 0;
};

class FlexProcessor : public BasebandProcessor {
public:
    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

private:
    static constexpr size_t baseband_fs = 2000000;
    static constexpr uint8_t stat_update_interval = 10;
    static constexpr uint32_t stat_update_threshold =
        baseband_fs / stat_update_interval;

    void configure();
    void flush();
    void reset();
    void send_stats() const;
    void send_packet();
    void on_beep_message(const AudioBeepMessage& message);

    bool configured = false;

    std::array<complex16_t, 256> dst{};
    const buffer_c16_t dst_buffer{dst.data(), dst.size()};

    std::array<float, 16> audio{};
    const buffer_f32_t audio_buffer{audio.data(), audio.size()};

    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};
    dsp::demodulate::FM demod{};

    AudioNormalizer normalizer{};
    AudioOutput audio_output{};

    FlexPacket packet{};

    uint32_t samples_processed = 0;

    BitQueue bits{};
    BitExtractor bit_extractor{bits};
    FrameExtractor frame_extractor{
        bits, [this](FrameExtractor&) {
            send_packet();
        }};

    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_FLEX_H__*/