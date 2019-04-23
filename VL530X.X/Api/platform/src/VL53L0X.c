#include <xc.h>
#include "VL53L0X.h"

void InitVL53L0X_I2C(VL53L0X_Dev_t *dev) {
    TRISDbits.TRISD5 = 1;
    TRISDbits.TRISD6 = 1;
    SSP2ADD = ((_XTAL_FREQ / 4) / (dev->comms_speed_khz * 1000L)) - 1; //0x18; //400kHz
    if (dev->comms_speed_khz == 400) {
        SSP2STATbits.SMP = 0; //0 = 400kHz 1 = 100kHz
    } else {
        SSP2STATbits.SMP = 1;
    }
    SSP2CON1bits.SSPM = 0b1000; //I2C Master mode
    SSP2CON1bits.SSPEN = 1; //Enable MSSP
    char temp = SSP2BUF; //Make sure buffer is clear
}

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = deviceAddress;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT == 1) {
        return -1; //No ACK from device
    }
    SSP2BUF = index;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    uint32_t i;
    for (i = 0; i < count; ++i) {
        SSP2BUF = *pdata;
        while (SSP2STATbits.BF || SSP2STATbits.R_W);
        ++pdata;
    }
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN == 1);
    return 0;
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = deviceAddress;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT == 1) {
        return -1; //No ACK from device
    }
    SSP2BUF = index;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RSEN = 1;
    while (SSP2CON2bits.RSEN == 1);
    SSP2BUF = deviceAddress | 1;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    uint32_t i;
    for (i = 0; i < count; ++i) {
        SSP2CON2bits.RCEN = 1;
        while (!SSP2STATbits.BF);
        *pdata = SSP2BUF;
        SSP2CON2bits.ACKDT = i < (count - 1) ? 0 : 1;
        SSP2CON2bits.ACKEN = 1;
        while (SSP2CON2bits.ACKEN != 0);
        ++pdata;
    }
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN == 1);
    return 0;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = deviceAddress;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT == 1) {
        return -1; //No ACK from device
    }
    SSP2BUF = index;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = data;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN == 1);
    return 0;
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = data >> 8;
    buffer[1] = data & 0x00ff;
    return VL53L0X_write_multi(deviceAddress, index, buffer, 2);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = data >> 24;
    buffer[1] = (data & 0x00ff0000L) >> 16;
    buffer[2] = (data & 0x0000ff00L) >> 8;
    buffer[3] = data & 0x000000ffL;
    return VL53L0X_write_multi(deviceAddress, index, buffer, 4);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = deviceAddress;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT == 1) {
        return -1; //No ACK from device
    }
    SSP2BUF = index;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RSEN = 1;
    while (SSP2CON2bits.RSEN == 1);
    SSP2BUF = deviceAddress | 1;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RCEN = 1;
    while (!SSP2STATbits.BF);
    *data = SSP2BUF;
    SSP2CON2bits.ACKDT = 1;
    SSP2CON2bits.ACKEN = 1;
    while (SSP2CON2bits.ACKEN != 0);
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN == 1);
    return 0;
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data) {
    int status;
    uint8_t buffer[2];
    status = VL53L0X_read_multi(deviceAddress, index, buffer, 2);
    if (status != 0) {
        return status;
    }
    *data = buffer[0];
    *data <<= 8;
    *data |= buffer[1];
    return 0;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data) {
    int status;
    uint8_t buffer[4];
    status = VL53L0X_read_multi(deviceAddress, index, buffer, 4);
    if (status != 0) {
        return status;
    }
    *data = buffer[0];
    *data <<= 8;
    *data |= buffer[1];
    *data <<= 8;
    *data |= buffer[2];
    *data <<= 8;
    *data |= buffer[3];
    return 0;
}

void VL53L0X_pollingDelay(void) {
    __delay_ms(5);
}

//char ReadRegister(char reg) {
//    char data;
//    SSP2CON2bits.SEN = 1; //Start condition
//    while (SSP2CON2bits.SEN == 1);
//    data = SSP2BUF; //Read SSPxBUF to make sure BF is clear
//    SSP2BUF = VL530X_ADDRESS; //address with R/W clear for write
//    while (SSP2STATbits.BF || SSP2STATbits.R_W);
//    SSP2BUF = reg; //Send register address
//    while (SSP2STATbits.BF || SSP2STATbits.R_W);
//    SSP2CON2bits.RSEN = 1; //Restart
//    while (SSP2CON2bits.RSEN == 1);
//    SSP2BUF = VL530X_ADDRESS | 1; //address with R/W set for read
//    while (SSP2STATbits.BF || SSP2STATbits.R_W);
//    SSP2CON2bits.RCEN = 1; // enable master for 1 byte reception
//    while (!SSP2STATbits.BF); // wait until byte received
//    data = SSP2BUF;
//    SSP2CON2bits.ACKDT = 1; //Last byte so NACK
//    SSP2CON2bits.ACKEN = 1; //Send ACK
//    while (SSP2CON2bits.ACKEN != 0);
//    SSP2CON2bits.PEN = 1; //Stop condition
//    while (SSP2CON2bits.PEN == 1);
//    return data;
//}
