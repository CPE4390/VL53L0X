#include <xc.h>
#include <stdio.h>
#include "LCD.h"
#include "VL53L0X.h"
#include "Api/core/inc/vl53l0x_api.h"
#include "Api/platform/inc/vl53l0x_platform.h"

#pragma config FOSC=HSPLL
#pragma config WDTEN=OFF
#pragma config XINST=OFF

//Pins
//SCL = RD6
//SDA = RD5
//INT = RB1

void InitPins(void);
void ConfigInterrupts(void);

VL53L0X_Dev_t dev;
VL53L0X_Version_t Version;
VL53L0X_DeviceInfo_t devInfo;

int getRange(void);
int doInit(void);

void main(void) {
    int status;
    long i;
    int count = 0;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    
    OSCTUNEbits.PLLEN = 1;
    InitPins();
    LCDInit();
    LCDClear();
    ConfigInterrupts();
    dev.I2cDevAddr = VL53L0X_ADDRESS;
    dev.comms_speed_khz = 400;
    InitVL53L0X_I2C(&dev);
    status = VL53L0X_DataInit(&dev);
    status = VL53L0X_GetDeviceInfo(&dev, &devInfo);
    lprintf(1, "status=%d", status);
    lprintf(0, "%s", devInfo.Name);
    status = doInit();
    if (status != VL53L0X_ERROR_NONE) {
        lprintf(1, "Error=%d", status);
        while(1);
    }
    while (1) {
        status = getRange();
        lprintf(1, "Range=%d mm", status);
        __delay_ms(300);
        ++count;
        if (count > 1000) {
            status = VL53L0X_PerformRefCalibration(&dev, &VhvSettings, &PhaseCal);
            count = 0;
        }
    }
}

int doInit(void) {
    uint32_t refSpadCount;
    uint8_t isAperatureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    int status;

    status = VL53L0X_StaticInit(&dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    status = VL53L0X_PerformRefSpadManagement(&dev, &refSpadCount, &isAperatureSpads);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    status = VL53L0X_PerformRefCalibration(&dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    status = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }
    return status;
}

int getRange(void) {
    int status;
    VL53L0X_RangingMeasurementData_t data;
    status = VL53L0X_PerformSingleRangingMeasurement(&dev, &data);
    if (status != VL53L0X_ERROR_NONE) {
        return -999;
    }
    return data.RangeMilliMeter;
}

void InitPins(void) {
    LATD = 0;
    TRISD = 0;
    WDTCONbits.ADSHR = 1;
    ANCON0 = ANCON1 = 0xff; //All digital
    WDTCONbits.ADSHR = 1;
}

void ConfigInterrupts(void) {

    RCONbits.IPEN = 0; //no priorities.  This is the default.
    //set up INT1 to interrupt on falling edge
    INTCON2bits.INTEDG1 = 0; //interrupt on falling edge
    INTCON3bits.INT1IP = 1;
    INTCON3bits.INT1IF = 0;
    INTCON3bits.INT1E = 1;

    //INTCONbits.GIE = 1; //Turn on interrupts
}

void __interrupt(high_priority) HighIsr(void) {

    //Check the source of the interrupt
    if (INTCON3bits.INT1IF == 1) {

        INTCON3bits.INT1IF = 0;
    }
}


