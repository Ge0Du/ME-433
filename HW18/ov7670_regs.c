#include <stdint.h>
#include "ov7670_regs.h"

//--------------------------------------------------------------------------------
// Definition of OV7670_init[92][2]
//--------------------------------------------------------------------------------
const uint8_t OV7670_init[92][2] = {
    {OV7670_REG_TSLB,       OV7670_TSLB_YLAST},  // No auto window
    {OV7670_REG_SLOP,       0x20},
    {OV7670_REG_GAM_BASE,   0x10},  // Gamma[0]
    {OV7670_REG_GAM_BASE + 1, 0x1e},// Gamma[1]
    {OV7670_REG_GAM_BASE + 2, 0x35},
    {OV7670_REG_GAM_BASE + 3, 0x5a},
    {OV7670_REG_GAM_BASE + 4, 0x69},
    {OV7670_REG_GAM_BASE + 5, 0x76},
    {OV7670_REG_GAM_BASE + 6, 0x80},
    {OV7670_REG_GAM_BASE + 7, 0x88},
    {OV7670_REG_GAM_BASE + 8, 0x8f},
    {OV7670_REG_GAM_BASE + 9, 0x96},
    {OV7670_REG_GAM_BASE + 10, 0xa3},
    {OV7670_REG_GAM_BASE + 11, 0xaf},
    {OV7670_REG_GAM_BASE + 12, 0xc4},
    {OV7670_REG_GAM_BASE + 13, 0xd7},
    {OV7670_REG_GAM_BASE + 14, 0xe8},
    {OV7670_REG_COM8,       (uint8_t)(OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_BANDING)},
    {OV7670_REG_GAIN,       0x00},
    {OV7670_REG_COM2,       0x00},
    {OV7670_REG_COM4,       0x40},
    {OV7670_REG_COM9,       0x20},  // Max AGC value
    {OV7670_REG_COM11,      (uint8_t)(1 << 3)}, // 50Hz banding
    {0x9D,                  0x59},  // (50 Hz filter at 13.888 MHz)
    {OV7670_REG_BD50MAX,    0x05},
    {OV7670_REG_BD60MAX,    0x07},
    {OV7670_REG_AEW,        0x95},
    {OV7670_REG_AEB,        0x33},
    {OV7670_REG_VPT,        0xe3},
    {OV7670_REG_HAECC1,     0x78},
    {OV7670_REG_HAECC2,     0x68},
    {0xA1,                  0x03},  // reserved
    {OV7670_REG_HAECC3,     0xd8},
    {OV7670_REG_HAECC4,     0xd8},
    {OV7670_REG_HAECC5,     0xf0},
    {OV7670_REG_HAECC6,     0x90},
    {OV7670_REG_HAECC7,     0x94},
    {OV7670_REG_COM5,       0x61},
    {OV7670_REG_COM6,       0x4B},
    {0x16,                  0x02},  // reserved
    {OV7670_REG_MVFP,       0x07},  // Mirror/flip
    {OV7670_REG_ADCCTR1,    0x02},
    {OV7670_REG_ADCCTR2,    0x91},
    {0x29,                  0x07},  // reserved
    {OV7670_REG_CHLF,       0x0B},
    {0x35,                  0x0B},  // reserved
    {OV7670_REG_ADC,        0x1D},
    {OV7670_REG_ACOM,       0x71},
    {OV7670_REG_OFON,       0x2A},
    {OV7670_REG_COM12,      0x78},
    {0x4D,                  0x40},  // reserved
    {0x4E,                  0x20},  // reserved
    {OV7670_REG_GFIX,       0x5D},
    {OV7670_REG_REG74,      0x19},
    {0x8D,                  0x4F},  // reserved
    {0x8E,                  0x00},  // reserved
    {0x8F,                  0x00},  // reserved
    {0x90,                  0x00},  // reserved
    {0x91,                  0x00},  // reserved
    {OV7670_REG_DM_LNL,     0x00},
    {0x96,                  0x00},  // reserved
    {0x9A,                  0x80},  // reserved
    {0xB0,                  0x84},  // reserved
    {OV7670_REG_ABLC1,      0x0C},
    {0xB2,                  0x0E},  // reserved
    {OV7670_REG_THL_ST,     0x82},
    {0xB8,                  0x0A},  // reserved
    {OV7670_REG_AWBC1,      0x14},
    {OV7670_REG_AWBC2,      0xF0},
    {OV7670_REG_AWBC3,      0x34},
    {OV7670_REG_AWBC4,      0x58},
    {OV7670_REG_AWBC5,      0x28},
    {OV7670_REG_AWBC6,      0x3A},
    {0x59,                  0x88},  // reserved
    {0x5A,                  0x88},  // reserved
    {0x5B,                  0x44},  // reserved
    {0x5C,                  0x67},  // reserved
    {0x5D,                  0x49},  // reserved
    {0x5E,                  0x0E},  // reserved
    {OV7670_REG_LCC3,       0x04},
    {OV7670_REG_LCC4,       0x20},
    {OV7670_REG_LCC5,       0x05},
    {OV7670_REG_LCC6,       0x04},
    {OV7670_REG_LCC7,       0x08},
    {OV7670_REG_AWBCTR3,    0x0A},
    {OV7670_REG_AWBCTR2,    0x55},
    {OV7670_REG_AWBCTR1,    0x11},
    {OV7670_REG_AWBCTR0,    0x9E},
    {OV7670_REG_BRIGHT,     0x00},
    {OV7670_REG_CONTRAS,    0x40},
    {OV7670_REG_CONTRAS_CENTER, 0x80}
};

//--------------------------------------------------------------------------------
// Definition of OV7670_rgb[12][2]
//--------------------------------------------------------------------------------
const uint8_t OV7670_rgb[12][2] = {
    {OV7670_REG_COM7,    OV7670_COM7_RGB},                           // RGB mode
    {OV7670_REG_RGB444,  0x00},                                      // disable RGB444
    {OV7670_REG_COM15,   (uint8_t)(OV7670_COM15_RGB565 | OV7670_COM15_R00FF)},   // full‐range RGB565
    {OV7670_REG_COM9,    0x6A},  // 128× AGC ceiling
    {0x4F,               0x80},  // “matrix coefficient 1”
    {0x50,               0x80},  // “matrix coefficient 2”
    {0x51,               0x00},  // “matrix coefficient 3”
    {0x52,               0x22},  // “matrix coefficient 4”
    {0x53,               0x5E},  // “matrix coefficient 5”
    {0x54,               0x80},  // “matrix coefficient 6”
    {OV7670_REG_COM13,   OV7670_COM13_UVSAT},                          // UV saturation
    {0xFF,               0xFF},  // end‐of‐list marker
};
