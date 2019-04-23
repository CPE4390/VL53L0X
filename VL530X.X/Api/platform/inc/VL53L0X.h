/* 
 * File:   VL530X.h
 * Author: Brad
 *
 * Created on April 7, 2017, 5:28 PM
 */

#ifndef VL53L0X_H
#define	VL53L0X_H

#include <stdint.h>
#include "vl53l0x_api.h"

//Pins
//SCL = RD6
//SDA = RD5
//INT = RB1

#define VL53L0X_ADDRESS     0x52
#define _XTAL_FREQ 40000000L

#ifdef	__cplusplus
extern "C" {
#endif
    void InitVL53L0X_I2C(VL53L0X_Dev_t *dev);
    //char ReadRegister(char reg);
    int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count);
    int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count);
    int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data);
    int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data);
    int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data);
    int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data);
    int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data);
    int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data);
    void VL53L0X_pollingDelay(void);

#ifdef	__cplusplus
}
#endif

#endif	/* VL530X_H */

