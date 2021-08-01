#include <stdint.h>
#include "input-event-codes.h"

uint8_t scancode_to_keycode[256] = {
    [0x0d] = KEY_TAB,
    [0x0e] = KEY_GRAVE,
    [0x12] = KEY_LEFTSHIFT,
    [0x14] = KEY_LEFTCTRL,
    [0x15] = KEY_Q,
    [0x16] = KEY_1,
    [0x1a] = KEY_Z,
    [0x1b] = KEY_S,
    [0x1c] = KEY_A,
    [0x1e] = KEY_2,
    [0x1d] = KEY_W,
    [0x21] = KEY_C,
    [0x22] = KEY_X,
    [0x23] = KEY_D,
    [0x24] = KEY_E,
    [0x25] = KEY_4,
    [0x26] = KEY_3,
    [0x29] = KEY_SPACE,
    [0x2a] = KEY_V,
    [0x2b] = KEY_F,
    [0x2c] = KEY_T,
    [0x2d] = KEY_R,
    [0x2e] = KEY_5,
    [0x31] = KEY_N,
    [0x32] = KEY_B,
    [0x33] = KEY_H,
    [0x34] = KEY_G,
    [0x35] = KEY_Y,
    [0x36] = KEY_6,
    [0x3a] = KEY_M,
    [0x3b] = KEY_J,
    [0x3c] = KEY_U,
    [0x3d] = KEY_7,
    [0x3e] = KEY_8,
    [0x41] = KEY_COMMA,
    [0x42] = KEY_K,
    [0x43] = KEY_I,
    [0x44] = KEY_O,
    [0x45] = KEY_0,
    [0x46] = KEY_9,
    [0x49] = KEY_DOT,
    [0x4a] = KEY_SLASH,
    [0x4b] = KEY_L,
    [0x4c] = KEY_SEMICOLON,
    [0x4d] = KEY_P,
    [0x4e] = KEY_MINUS,
    [0x52] = KEY_APOSTROPHE,
    [0x54] = KEY_LEFTBRACE,
    [0x58] = KEY_LEFTCTRL,      // map capslock to left ctrl
    [0x5b] = KEY_RIGHTBRACE,
    [0x59] = KEY_RIGHTSHIFT,
    [0x5a] = KEY_ENTER,
    [0x5d] = KEY_BACKSLASH,
    [0x66] = KEY_BACKSPACE,
    [0x6b] = KEY_LEFT,
    [0x72] = KEY_DOWN,
    [0x74] = KEY_RIGHT,
    [0x75] = KEY_UP,
    [0x76] = KEY_ESC,
};