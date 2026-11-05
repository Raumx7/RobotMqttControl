#pragma once

#include <WiFiType.h>
#include "types.h"

// OLED dimentions (128x64) with 5x7 text rendering
constexpr uint8_t FONT_WIDTH = 6;   // 5 + 1 spacing
constexpr uint8_t FONT_HEIGH = 8;   // 7 + 1 spacing

// Max columns and rows
constexpr uint8_t MAX_COL = 128 / FONT_WIDTH;   // max col: 128 / 6 = 21
constexpr uint8_t MAX_ROW = 64 / FONT_HEIGH;    // max row: 64 / 8 = 8

// Pixels to row number in range: [0, MAX_ROW - 1]
inline constexpr uint8_t ROW[] {
    0 * FONT_HEIGH,
    1 * FONT_HEIGH,
    2 * FONT_HEIGH,
    3 * FONT_HEIGH,
    4 * FONT_HEIGH,
    5 * FONT_HEIGH,
    6 * FONT_HEIGH,
    7 * FONT_HEIGH
};

// Pixels to column number in range: [0, MAX_COL - 1]
inline constexpr uint8_t COL[] {
    0 * FONT_WIDTH,
    1 * FONT_WIDTH,
    2 * FONT_WIDTH,
    3 * FONT_WIDTH,
    4 * FONT_WIDTH,
    5 * FONT_WIDTH,
    6 * FONT_WIDTH,
    7 * FONT_WIDTH,
    8 * FONT_WIDTH,
    9 * FONT_WIDTH,
    10 * FONT_WIDTH,
    11 * FONT_WIDTH,
    12 * FONT_WIDTH,
    13 * FONT_WIDTH,
    14 * FONT_WIDTH,
    15 * FONT_WIDTH,
    16 * FONT_WIDTH,
    17 * FONT_WIDTH,
    18 * FONT_WIDTH,
    19 * FONT_WIDTH,
    20 * FONT_WIDTH,
    21 * FONT_WIDTH,
};

// Converts robot-type, wifi or mqtt client status to C-String
namespace fmt {
    namespace detail {
        inline constexpr const char* BrakeModeStr[] {
            "coast",
            "brake"
        };

        inline constexpr const char* CmdTypeStr[] {
            "arcade",
            "move",
            "square",
            "set_brake",
            "stop",
            "rectangle_scan"
        };

        inline constexpr const char* CmdErrStr[] {
            "command_unknown",
            "already_executed",
            "autonomous_running",
            "invalid_argument"
        };
    }

    constexpr const char* to_str(BrakeMode brake_mode) {
        return detail::BrakeModeStr[static_cast<uint8_t>(brake_mode)];
    }

    constexpr const char* to_str(CommandType command_type) {
        return detail::CmdTypeStr[static_cast<uint8_t>(command_type)];
    }

    constexpr const char* to_str(CommandError command_error) {
        return detail::CmdErrStr[static_cast<uint8_t>(command_error)];
    }

    constexpr const char* to_str(AutoState auto_state) {
        return auto_state != AutoState::Idle ? "autonomous" : "manual";
    }

    constexpr const char* to_str(int client_state) {
        return client_state != MQTT_CONNECTED ? "offline" : "online";
    }

    constexpr const char* to_str(wl_status_t wifi_status) {
        return wifi_status != WL_CONNECTED ? "disconnected" : "connected";
    }
}

// Status text indicators (displayed on OLED)
namespace text {
    inline constexpr auto line = "---------------------";
    inline constexpr auto wifi = "  CONNECTION STATUS";
    inline constexpr auto data = "    SENSOR DATA";
}
