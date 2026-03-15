#pragma once
// ============================================================================
//  theme.h — UI color theme constants
// ============================================================================
//
// All hex color values used across the LVGL interface.
// Change these to restyle the entire UI.

#include <cstdint>

namespace Theme {
    static constexpr uint32_t BG        = 0x000000;  // screen background
    static constexpr uint32_t NAV_BG    = 0x0C0C0C;  // navigation bar
    static constexpr uint32_t CARD      = 0x242424;  // card backgrounds
    static constexpr uint32_t BTN_DARK  = 0x3A3A3A;  // dark buttons (decrease, back)
    static constexpr uint32_t BTN_MID   = 0x2A2A2A;  // medium buttons (back nav)
    static constexpr uint32_t ACCENT    = 0x1F6FEB;  // primary blue accent
    static constexpr uint32_t GREEN     = 0x00FF00;  // run button, active values
    static constexpr uint32_t RED       = 0xFF4444;  // stop button, warnings
    static constexpr uint32_t TEXT      = 0xFFFFFF;  // primary text
    static constexpr uint32_t MUTED     = 0xCFCFCF;  // secondary text
    static constexpr uint32_t DIM       = 0xAAAAAA;  // tertiary text (subtitle)
    static constexpr uint32_t HINT      = 0x888888;  // hint text, inactive
    static constexpr uint32_t JAM_MSG   = 0xCCCCCC;  // jam screen body text
    static constexpr uint32_t WARN_BG   = 0x3A2B12;  // warning badge background
    static constexpr uint32_t WARN_TEXT = 0xFFD37C;  // warning text
    static constexpr uint32_t SPEED_TXT = 0x6FA8FF;  // speed profile indicator
    static constexpr uint32_t CAL_BTN   = 0x444444;  // calibrate button
}
