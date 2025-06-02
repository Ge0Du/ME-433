#ifndef OV7670_REGS_H
#define OV7670_REGS_H

#include <stdint.h>

// -------------------------------------------------------------------------
// === OV7670 register‐list declarations ===
// These arrays must be defined exactly once (in ov7670_regs.c).
// -------------------------------------------------------------------------

extern const uint8_t OV7670_init[93][2];
extern const uint8_t OV7670_rgb[3][2];

#endif  // OV7670_REGS_H
