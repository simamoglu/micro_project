#ifndef __DS1307_H__
#define __DS1307_H__

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define DS1307_ADDRESS (0x68 << 1)

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} RTC_Time;

bool DS1307_Init(I2C_HandleTypeDef *hi2c);
bool DS1307_GetTime(I2C_HandleTypeDef *hi2c, RTC_Time *time);
uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);

#endif
