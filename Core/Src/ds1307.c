#include "ds1307.h"

#define DS1307_ADDRESS 0x68 << 1

uint8_t bcd2dec(uint8_t val) {
    return ((val / 16 * 10) + (val % 16));
}

uint8_t dec2bcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

bool DS1307_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    return (HAL_I2C_IsDeviceReady(hi2c, DS1307_ADDRESS, 1, 100) == HAL_OK);
}

bool DS1307_GetTime(I2C_HandleTypeDef *hi2c, RTC_Time *time) {
    uint8_t data[7];
    uint8_t reg = 0x00;

    if (HAL_I2C_Master_Transmit(hi2c, DS1307_ADDRESS, &reg, 1, 100) != HAL_OK)
        return false;

    if (HAL_I2C_Master_Receive(hi2c, DS1307_ADDRESS, data, 7, 100) != HAL_OK)
        return false;

    time->seconds     = bcd2dec(data[0] & 0x7F);
    time->minutes     = bcd2dec(data[1]);
    time->hours       = bcd2dec(data[2] & 0x3F);
    time->day_of_week = bcd2dec(data[3]);
    time->date        = bcd2dec(data[4]);
    time->month       = bcd2dec(data[5]);
    time->year        = bcd2dec(data[6]);

    return true;
}
