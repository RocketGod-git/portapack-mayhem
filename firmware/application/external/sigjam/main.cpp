#include "ui.hpp"
#include "ui_sigjam.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::sigjam {
void initialize_app(ui::NavigationView& nav) {
    nav.push<SigJamView>();
}
}  // namespace ui::external_app::sigjam

extern "C" {
__attribute__((section(".external_app.app_sigjam.application_information"), used)) application_information_t _application_information_sigjam = {
    (uint8_t*)0x00000000,
    ui::external_app::sigjam::initialize_app,
    CURRENT_HEADER_VERSION,
    VERSION_MD5,
    "SigJam",
    {
        0xFF,
        0xFF,
        0xC3,
        0xC3,
        0x99,
        0x99,
        0x99,
        0x99,
        0x99,
        0x99,
        0xC3,
        0xC3,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xC3,
        0xC3,
        0x99,
        0x99,
        0x99,
        0x99,
        0x99,
        0x99,
        0xC3,
        0xC3,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
    },
    ui::Color::blue().v,
    app_location_t::TX,
    -1,
    {'P', 'J', 'A', 'M'},
    0x00000000,
};
}