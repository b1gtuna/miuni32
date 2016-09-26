#include "keymap_common.h"
const action_t PROGMEM fn_actions[] = {};
const uint8_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /* 0: qwerty */
    KEYMAP(Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,   BSPC, \
           A,   S,   D,   F,   G,   H,   J,   K,   L,   RSFT,ENT, \
           Z,   X,   C,   V,   SPC,      B,   N,   M ,  RALT, RCTL ),
};
