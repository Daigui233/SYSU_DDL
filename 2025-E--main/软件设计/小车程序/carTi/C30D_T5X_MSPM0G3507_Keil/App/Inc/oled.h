#ifndef OLED_H
#define OLED_H

#include <stdbool.h>

/*
 * SSD1306 128x64 OLED on I2C1:
 *   PB2 = SCL, PB3 = SDA, 7-bit address = 0x3C.
 */
#define OLED_I2C_ADDRESS 0x3CU

bool oled_init(void);
bool oled_is_ready(void);

/*
 * Update the RAM image. oled_process() sends one page per call so refreshing
 * the display never holds the 100 Hz motor loop for a complete screen write.
 */
void oled_show_status(float roll_deg, float pitch_deg, float yaw_deg,
                      float left_mps, float right_mps, bool imu_valid);
void oled_process(void);

#endif
