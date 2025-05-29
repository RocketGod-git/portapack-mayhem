#pragma once

#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_tabview.hpp"
#include "transmitter_model.hpp"
#include "message.hpp"
#include "jammer.hpp"
#include "lfsr_random.hpp"
#include "radio_state.hpp"

namespace ui::external_app::sigjam {

class RangeView : public View {
   public:
    RangeView(NavigationView& nav);
    void focus() override;
    void paint(Painter&) override;

    jammer::jammer_range_t frequency_range{false, 0, 0};

   private:
    void update_start(rf::Frequency f);
    void update_stop(rf::Frequency f);
    void update_center(rf::Frequency f);
    void update_width(uint32_t w);

    uint32_t width{};
    rf::Frequency center{};

    Labels labels{
        {{16, 68}, "Start", Theme::getInstance()->fg_cyan->foreground},
        {{184, 68}, "Stop", Theme::getInstance()->fg_cyan->foreground},
        {{96, 36}, "Center", Theme::getInstance()->fg_cyan->foreground},
        {{100, 108}, "Width", Theme::getInstance()->fg_cyan->foreground}};

    Checkbox check_enabled{
        {8, 4},
        12,
        "Enable Range"};

    Button button_start{
        {0, 88, 88, 32},
        ""};
    Button button_stop{
        {152, 88, 88, 32},
        ""};
    Button button_center{
        {60, 56, 120, 32},
        ""};
    Button button_width{
        {60, 128, 120, 32},
        ""};
};

class SigJamView : public View {
   public:
    SigJamView(NavigationView& nav);
    ~SigJamView();

    SigJamView(const SigJamView&) = delete;
    SigJamView(SigJamView&&) = delete;
    SigJamView& operator=(const SigJamView&) = delete;
    SigJamView& operator=(SigJamView&&) = delete;

    void focus() override;
    std::string title() const override { return "SigJam TX"; };

   private:
    NavigationView& nav_;
    TxRadioState radio_state_{
        0,
        3500000,
        3072000};

    void start_tx();
    void on_timer();
    void stop_tx();
    void set_jammer_channel(uint32_t i, uint32_t width, uint64_t center, uint32_t duration);
    void on_retune(const rf::Frequency freq, const uint32_t range);
    void update_visuals();

    bool jamming{false};
    bool cooling{false};
    uint16_t seconds{0};
    int16_t mscounter{0};
    lfsr_word_t lfsr_v{1};

    RangeView view_range_a{nav_};
    RangeView view_range_b{nav_};
    RangeView view_range_c{nav_};

    std::array<RangeView*, 3> range_views{{&view_range_a, &view_range_b, &view_range_c}};

    TabView tab_view{
        {"Range 1", Theme::getInstance()->fg_blue->foreground, range_views[0]},
        {"Range 2", Theme::getInstance()->fg_blue->foreground, range_views[1]},
        {"Range 3", Theme::getInstance()->fg_blue->foreground, range_views[2]},
    };

    Labels labels{
        {{16, 184}, "Mode:", Theme::getInstance()->fg_cyan->foreground},
        {{16, 204}, "Speed:", Theme::getInstance()->fg_cyan->foreground},
        {{16, 224}, "Hop:", Theme::getInstance()->fg_cyan->foreground},
        {{16, 244}, "TX:", Theme::getInstance()->fg_cyan->foreground},
        {{16, 264}, "Pause:", Theme::getInstance()->fg_cyan->foreground},
        {{16, 284}, "Jitter:", Theme::getInstance()->fg_cyan->foreground},
        {{88, 244}, "s", Theme::getInstance()->fg_cyan->foreground},
        {{88, 264}, "s", Theme::getInstance()->fg_cyan->foreground},
        {{88, 284}, "/60", Theme::getInstance()->fg_cyan->foreground},
        {{152, 184}, "Gain:", Theme::getInstance()->fg_cyan->foreground},
        {{208, 184}, "Amp:", Theme::getInstance()->fg_cyan->foreground}};

    OptionsField options_mode{
        {56, 184},
        16,
        {{"OOK 650kHz", 0},
         {"2FSK 2.38kHz", 1},
         {"2FSK 47.6kHz", 2},
         {"MSK 99.97Kb/s", 3},
         {"GFSK 9.99Kb/s", 4},
         {"Bruteforce", 5},
         {"Sine Wave", 6},
         {"Square Wave", 7},
         {"Sawtooth Wave", 8},
         {"White Noise", 9},
         {"Triangle Wave", 10},
         {"Chirp Signal", 11},
         {"Gaussian Noise", 12},
         {"Burst Mode", 13}}};

    OptionsField options_speed{
        {64, 204},
        6,
        {{"10Hz  ", 10},
         {"100Hz ", 100},
         {"1kHz  ", 1000},
         {"10kHz ", 10000},
         {"100kHz", 100000}}};

    OptionsField options_hop{
        {56, 224},
        5,
        {{"10ms ", 1},
         {"50ms ", 5},
         {"100ms", 10},
         {"1s   ", 100},
         {"2s   ", 200}}};

    NumberField field_timetx{
        {56, 244},
        3,
        {1, 180},
        1,
        ' '};

    NumberField field_timepause{
        {64, 264},
        2,
        {1, 60},
        1,
        ' '};

    NumberField field_jitter{
        {64, 284},
        2,
        {1, 60},
        1,
        ' '};

    NumberField field_gain{
        {192, 184},
        2,
        {0, 47},
        1,
        ' '};

    NumberField field_amp{
        {232, 184},
        1,
        {0, 1},
        1,
        ' '};

    Button button_transmit{
        {152, 224, 80, 80},
        "START"};

    MessageHandlerRegistration message_handler_retune{
        Message::ID::Retune,
        [this](Message* const p) {
            const auto message = static_cast<const RetuneMessage*>(p);
            this->on_retune(message->freq, message->range);
        }};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_timer();
        }};
};

}  // namespace ui::external_app::sigjam