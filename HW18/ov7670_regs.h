#ifndef OV7670_REGS_H
#define OV7670_REGS_H

#include <stdint.h>
#include "ov7670.h"

//--------------------------------------------------------------------------------
// We moved the “big” array‐initializers into ov7670_regs.c.  Here we only
// declare them as extern so that the linker can resolve them.
//
//   OV7670_init  must have exactly 92 entries.
//   OV7670_rgb   must have exactly 12 entries.
//
//--------------------------------------------------------------------------------
extern const uint8_t OV7670_init[92][2];
extern const uint8_t OV7670_rgb[12][2];

#endif // OV7670_REGS_H
