#include "ui_sigjam.hpp"
#include "ui_receiver.hpp"
#include "ui_freqman.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"

namespace ui::external_app::sigjam {

void RangeView::focus() {
    check_enabled.focus();
}

void RangeView::update_start(rf::Frequency f) {
    frequency_range.min = f;
    button_start.set_text(to_string_short_freq(f));
    center = (frequency_range.min + frequency_range.max) / 2;
    width = abs(frequency_range.max - frequency_range.min);
    button_center.set_text(to_string_short_freq(center));
    button_width.set_text(to_string_short_freq(width));
}

void RangeView::update_stop(rf::Frequency f) {
    frequency_range.max = f;
    button_stop.set_text(to_string_short_freq(f));
    center = (frequency_range.min + frequency_range.max) / 2;
    width = abs(frequency_range.max - frequency_range.min);
    button_center.set_text(to_string_short_freq(center));
    button_width.set_text(to_string_short_freq(width));
}

void RangeView::update_center(rf::Frequency f) {
    center = f;
    button_center.set_text(to_string_short_freq(center));
    rf::Frequency min = center - (width / 2);
    rf::Frequency max = min + width;
    frequency_range.min = min;
    button_start.set_text(to_string_short_freq(min));
    frequency_range.max = max;
    button_stop.set_text(to_string_short_freq(max));
}

void RangeView::update_width(uint32_t w) {
    width = w;
    button_width.set_text(to_string_short_freq(width));
    rf::Frequency min = center - (width / 2);
    rf::Frequency max = min + width;
    frequency_range.min = min;
    button_start.set_text(to_string_short_freq(min));
    frequency_range.max = max;
    button_stop.set_text(to_string_short_freq(max));
}

void RangeView::paint(Painter& painter) {
    Rect r;
    Point p;
    Coord c;
    r = button_center.screen_rect();
    p = r.center() + Point(0, r.height() / 2);
    portapack::display.draw_line(p, p + Point(0, 20), Theme::getInstance()->fg_cyan->foreground);
    r = button_width.screen_rect();
    c = r.top() + (r.height() / 2);
    p = {r.left() - 40, c};
    portapack::display.draw_line({r.left(), c}, p, Theme::getInstance()->fg_cyan->foreground);
    portapack::display.draw_line(p, p + Point(10, -10), Theme::getInstance()->fg_cyan->foreground);
    portapack::display.draw_line(p, p + Point(10, 10), Theme::getInstance()->fg_cyan->foreground);
    p = {r.right() + 40, c};
    portapack::display.draw_line({r.right(), c}, p, Theme::getInstance()->fg_cyan->foreground);
    portapack::display.draw_line(p, p + Point(-10, -10), Theme::getInstance()->fg_cyan->foreground);
    portapack::display.draw_line(p, p + Point(-10, 10), Theme::getInstance()->fg_cyan->foreground);
    portapack::display.fill_rectangle({0, 0, 240, 320}, Theme::getInstance()->bg_darkest->background);
    portapack::display.fill_rectangle({4, 4, 232, 312}, Theme::getInstance()->fg_blue->foreground);
}

RangeView::RangeView(NavigationView& nav) {
    hidden(true);
    add_children({&labels,
                  &check_enabled,
                  &button_start,
                  &button_stop,
                  &button_center,
                  &button_width});
    check_enabled.on_select = [this](Checkbox&, bool v) {
        frequency_range.enabled = v;
    };
    button_start.on_select = [this, &nav](Button& button) {
        auto new_view = nav.push<FrequencyKeypadView>(frequency_range.min);
        new_view->on_changed = [this, &button](rf::Frequency f) {
            update_start(f);
        };
    };
    button_stop.on_select = [this, &nav](Button& button) {
        auto new_view = nav.push<FrequencyKeypadView>(frequency_range.max);
        new_view->on_changed = [this, &button](rf::Frequency f) {
            update_stop(f);
        };
    };
    button_center.on_select = [this, &nav](Button& button) {
        auto new_view = nav.push<FrequencyKeypadView>(center);
        new_view->on_changed = [this, &button](rf::Frequency f) {
            update_center(f);
        };
    };
    button_width.on_select = [this, &nav](Button& button) {
        auto new_view = nav.push<FrequencyKeypadView>(width);
        new_view->on_changed = [this, &button](rf::Frequency f) {
            update_width(f);
        };
    };
}

void SigJamView::focus() {
    tab_view.focus();
}

SigJamView::~SigJamView() {
    portapack::transmitter_model.disable();
    baseband::shutdown();
}

void SigJamView::on_retune(const rf::Frequency freq, const uint32_t) {
    if (freq) {
        portapack::transmitter_model.set_target_frequency(freq);
    }
}

void SigJamView::set_jammer_channel(uint32_t i, uint32_t width, uint64_t center, uint32_t duration) {
}

void SigJamView::start_tx() {
    jamming = true;
    button_transmit.set_style(Theme::getInstance()->fg_red);
    button_transmit.set_text("STOP");
    portapack::transmitter_model.set_rf_amp(field_amp.value());
    portapack::transmitter_model.set_tx_gain(field_gain.value());
    portapack::transmitter_model.set_baseband_bandwidth(28000000);
    portapack::transmitter_model.enable();
    baseband::set_jammer(true, (jammer::JammerType)options_mode.selected_index(), options_speed.selected_index_value());
    mscounter = 0;
    update_visuals();
}

void SigJamView::stop_tx() {
    button_transmit.set_style(Theme::getInstance()->fg_green);
    button_transmit.set_text("START");
    portapack::transmitter_model.disable();
    baseband::set_jammer(false, jammer::JammerType::TYPE_FSK, 0);
    jamming = false;
    cooling = false;
    update_visuals();
}

void SigJamView::on_timer() {
    if (++mscounter == 60) {
        mscounter = 0;
        if (jamming) {
            if (cooling) {
                if (++seconds >= field_timepause.value()) {
                    portapack::transmitter_model.set_baseband_bandwidth(28000000);
                    portapack::transmitter_model.enable();
                    button_transmit.set_style(Theme::getInstance()->fg_red);
                    button_transmit.set_text("STOP");
                    baseband::set_jammer(true, (jammer::JammerType)options_mode.selected_index(), options_speed.selected_index_value());
                    int32_t jitter_amount = field_jitter.value();
                    if (jitter_amount) {
                        lfsr_v = lfsr_iterate(lfsr_v);
                        jitter_amount = (jitter_amount / 2) - (lfsr_v & jitter_amount);
                        mscounter += jitter_amount;
                    }
                    cooling = false;
                    seconds = 0;
                    update_visuals();
                }
            } else {
                if (++seconds >= field_timetx.value()) {
                    portapack::transmitter_model.disable();
                    button_transmit.set_style(Theme::getInstance()->fg_yellow);
                    button_transmit.set_text("PAUSED");
                    baseband::set_jammer(false, jammer::JammerType::TYPE_FSK, 0);
                    int32_t jitter_amount = field_jitter.value();
                    if (jitter_amount) {
                        lfsr_v = lfsr_iterate(lfsr_v);
                        jitter_amount = (jitter_amount / 2) - (lfsr_v & jitter_amount);
                        mscounter += jitter_amount;
                    }
                    cooling = true;
                    seconds = 0;
                    update_visuals();
                }
            }
        }
    }
}

void SigJamView::update_visuals() {
    portapack::display.fill_rectangle({0, 0, 240, 320}, Theme::getInstance()->bg_darkest->background);
    portapack::display.fill_rectangle({4, 4, 232, 312}, jamming ? Theme::getInstance()->fg_red->foreground : Theme::getInstance()->fg_blue->foreground);
    if (jamming || cooling) {
        portapack::display.fill_rectangle({8, 8, 224, 40}, Theme::getInstance()->bg_medium->background);
        Painter painter;
        painter.draw_string({16, 24}, *Theme::getInstance()->fg_yellow, "SigJam Active");
    }
}

SigJamView::SigJamView(NavigationView& nav) : nav_{nav} {
    Rect view_rect = {0, 24, 240, 120};
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());
    add_children({&tab_view,
                  &view_range_a,
                  &view_range_b,
                  &view_range_c,
                  &labels,
                  &options_mode,
                  &options_speed,
                  &options_hop,
                  &field_timetx,
                  &field_timepause,
                  &field_jitter,
                  &field_gain,
                  &field_amp,
                  &button_transmit});
    view_range_a.set_parent_rect(view_rect);
    view_range_b.set_parent_rect(view_rect);
    view_range_c.set_parent_rect(view_rect);
    options_mode.set_selected_index(0);
    options_speed.set_selected_index(3);
    options_hop.set_selected_index(1);
    button_transmit.set_style(Theme::getInstance()->fg_green);
    field_timetx.set_value(30);
    field_timepause.set_value(1);
    field_jitter.set_value(0);
    field_gain.set_value(portapack::transmitter_model.tx_gain());
    field_amp.set_value(portapack::transmitter_model.rf_amp());
    button_transmit.on_select = [this](Button&) {
        if (jamming || cooling)
            stop_tx();
        else
            start_tx();
    };
    update_visuals();
}

}  // namespace ui::external_app::sigjam