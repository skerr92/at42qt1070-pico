/**
 * Copyright (c) 2022 Seth Kerr.
 *
 * SPDX-License-Identifier: MIT
 **/

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

/* Example code to talk to a AT42QT1070 Capacitive Touch sensor
    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.
    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.
    Connections on Raspberry Pi Pico board, other boards may vary.
    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on AT42QT1070
    board
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> SCL on
    AT42QT1070 board
    3.3v (pin 36) -> VCC on AT42QT1070 board
    GND (pin 38)  -> GND on AT42QT1070 board
 */

#define ADDR _u(0x1B)

enum chip_info {
  AT42QT107_CHIP_ID = _u(0x00),  // Chip ID should be 0x2F
  AT42QT107_FIRMWARE = _u(0x01), // get firmware version number
  AT42QT107_DETECT_STATUS = _u(0x02),
  // BIT 7 (CALIBRATE) | BIT6 (OVERFLOW) | - | - | - | - | - | TOUCH
}

enum {
  AT42QT107_KEY_STATUS =
      (0x03), // BIT 7 Reserved, BIT 6 (KEY 6) | ... | ... | BIT 0 (Key 0)
  AT42QT107_KEY_0_1 = (0x04), // Most Significant byte
  AT42QT107_KEY_0_2 = (0x05), // Least Significant byte
  AT42QT107_KEY_1_1 = (0x06), // MSByte
  AT42QT107_KEY_1_2 = (0x07), // LSByte
  AT42QT107_KEY_2_1 = (0x08),
  AT42QT107_KEY_2_2 = (0x09),
  AT42QT107_KEY_3_1 = (0x0A),
  AT42QT107_KEY_3_2 = (0x0B),
  AT42QT107_KEY_4_1 = (0x0C),
  AT42QT107_KEY_4_2 = (0x0D),
  AT42QT107_KEY_5_1 = (0x0E),
  AT42QT107_KEY_5_2 = (0x0F),
  AT42QT107_KEY_6_1 = (0x10),
  AT42QT107_KEY_6_2 = (0x11),

  AT42QT107_REF_DATA_0_1 = (0x12), // MSByte
  AT42QT107_REF_DATA_0_2 = (0x13), // LSByte
  AT42QT107_REF_DATA_1_1 = (0x14),
  AT42QT107_REF_DATA_1_2 = (0x15),
  AT42QT107_REF_DATA_2_1 = (0x16),
  AT42QT107_REF_DATA_2_2 = (0x17),
  AT42QT107_REF_DATA_3_1 = (0x18),
  AT42QT107_REF_DATA_3_2 = (0x19),
  AT42QT107_REF_DATA_4_1 = (0x1A),
  AT42QT107_REF_DATA_4_2 = (0x1B),
  AT42QT107_REF_DATA_5_1 = (0x1C),
  AT42QT107_REF_DATA_5_2 = (0x1D),
  AT42QT107_REF_DATA_6_1 = (0x1E),
  AT42QT107_REF_DATA_6_2 = (0x1F),

  AT42QT107_NTHR_K0 = (0x20), // negative threshold value.
  AT42QT107_NTHR_K1 = (0x21), // Do not set this value to 0
  AT42QT107_NTHR_K2 = (0x22), // 0 will cause the key to go into detection
  AT42QT107_NTHR_K3 = (0x23),
  AT42QT107_NTHR_K4 = (0x24),
  AT42QT107_NTHR_K5 = (0x25),
  AT42QT107_NTHR_K6 = (0x26),

  AT42QT107_AVE_KS_K0 = (0x27), // BIT 7 - BIT 3: Average factoring
  AT42QT107_AVE_KS_K1 = (0x28), // BIT 2 - BIT 0: Adjacent Key Supression
  AT42QT107_AVE_KS_K2 = (0x29), // AVE values set ADC Sample Number
  AT42QT107_AVE_KS_K3 =
      (0x2A), // AVE can be: 1, 2, 4, 8, 16, 32 ONLY, Default of 8.
  AT42QT107_AVE_KS_K4 = (0x2B), // AKS bits can have a value between 0 and 3
  AT42QT107_AVE_KS_K5 = (0x2C), // AKS value of 0 means key is not part of a
  AT42QT107_AVE_KS_K6 = (0x2D), // AKS group. Default AVE/AKS value 0x01

  AT42QT107_DI_K0 = (0x2E), // Detection integrator
  AT42QT107_DI_K1 = (0x2F), // 8 bit value controls the number of consecutive
  AT42QT107_DI_K2 = (0x30), // measurements that must be confirmed to having
  AT42QT107_DI_K3 =
      (0x31), // passed the key threshold for a key being registered
  AT42QT107_DI_K4 = (0x32),    // as a detect.
  AT42QT107_DI_K5 = (0x33),    // Minimum value for DI filter is 2
  AT42QT107_DI_K6 = (0x34),    // Default of 4
  AT42QT107_FO_MO_GA = (0x35), // FastOutDI/ Max Cal/ Guard Channel
  // Fast Out DI (FO Mode) - BIT 5 is set, filters with integrator of 4.
  // MAX CAL: if cleared, all keys recalibrated after Max On Duration timeout
  // GUARD CHANNEL: bits 0-3 are used to set a key as the guard channel. Valid
  // values 0-6, with any larger value disabling the guard key feature.

  AT42QT107_CAL = (0x38), // Calibrate by writing any value not equal to zero
  AT42QT107_RESET =
      (0x39), // active low reset, write any nonzero value to reset
};

enum power_regs {
  AT42QT107_LP = (0x36), // lower power mode

  // 8 bit value, 0 = 8ms between samples
  // 1 = 8ms | 2 = 16ms | 3 = 24ms | 4 = 32 | 254 = 2.032s | 255 = 2.040s
  // Default value: 2 (16ms)

  AT42QT107_MAX_ON_DUR = (0x37), // maximum on duration

  // 8 bit value to determine how long any key can be in touch before it
  // recalibrates itself 0 = off | 1 = 160ms | 2 = 320ms | 3 = 480ms | 4 = 640ms
  // | 255 = 40.8s Default value: 180(160ms*180 = 28.8s)

};

#ifdef i2c_default
void at42qt1070_init() {
  uint8_t buf[2];

  const uint8_t reg_config_val = (0x01 & 0xFF);

  buf[0] = AT42QT107_CAL;
  buf[1] = reg_config_val;
  i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

}
void at42qt1070_reset() {
  uint8_t buf[2] = { AT42QT107_RESET, 0x01 };
  i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

void at42qt1070_set_lp(uint8_t value) {
  uint8_t buf[2] = { AT42QT107_LP, value);
  i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

bool at42qt1070_touched() {
  
}
