/*    __  __ _____ _____   ____  __  __ _____ _____ ____
 *   |  \/  |_   _|  __ \ / __ \|  \/  |_   _/ ____/ __ \
 *   | \  / | | | | |__) | |  | | \  / | | || |   | |  | |
 *   | |\/| | | | |  _  /| |  | | |\/| | | || |   | |  | |
 *   | |  | |_| |_| | \ \| |__| | |  | |_| || |___| |__| |
 *   |_|  |_|_____|_|  \_\\____/|_|  |_|_____\_____\____/
 *
 * Copyright (c) 2019 Miromico AG
 * All rights reserved.
 */

#include "lmic.h"
#include "hw.h"
#include "i2c.h"

#include "sht3x.h"

static struct {
    unsigned char buf[6];

    osjobcb_t cb;

    eDeviceStatus_t* pstatus;
    sht3x_data* pdata;
} sht3x;

#define SHT3X_ADDRESS_L   0x44
#define SHT3X_ADDRESS_H   0x45

static const uint8_t SHT3X_ADDRESS = (SHT3X_ADDRESS_H << 1);

static const uint8_t CMD_READ_STATUS_REG[] = { 0xF3, 0x2D };
static const uint8_t CMD_MEASURE_HPM[]     = { 0x24, 0x00 };
static const uint8_t DELAY_MEASURE_HPM     = 15;

static uint8_t sht3x_generate_crc(uint8_t* data, uint16_t count);
static inline bool sht3x_check_crc(uint8_t* data, uint16_t count);

#define I2C_TIMEOUT   ms2osticks(500)

static void init_func(osjob_t* job) {
    static enum {
        INIT, CFG_DONE
    } state;
    static int i2c_status;

    switch (state) {
    default:
    case INIT:
        sht3x.buf[0] = CMD_READ_STATUS_REG[0];
        sht3x.buf[1] = CMD_READ_STATUS_REG[1];

        i2c_xfer_ex(SHT3X_ADDRESS, sht3x.buf, 2, 3, I2C_TIMEOUT, job, init_func,
                    &i2c_status);
        break;

    case CFG_DONE:
        if (i2c_status == I2C_OK) {
            if (sht3x_check_crc(sht3x.buf, 2)) {
                *sht3x.pstatus = eDevice_Initialized;
            } else {
                *sht3x.pstatus = eDevice_Failed;
            }
        } else {
            *sht3x.pstatus = eDevice_Failed;
        }

        // done -- invoke application callback
        os_setCallback(job, sht3x.cb);
        state = INIT;
        return;
    }
    state += 1;
}

static void read_func(osjob_t* job) {
    static enum {
        INIT, TRIGGERED, READ, DONE,
    } state;
    static int i2c_status;

    switch (state) {
    default:
    case INIT: {
        // Trigger measurement
        sht3x.buf[0] = CMD_MEASURE_HPM[0];
        sht3x.buf[1] = CMD_MEASURE_HPM[1];

        i2c_xfer_ex(SHT3X_ADDRESS, sht3x.buf, 2, 0, I2C_TIMEOUT, job, read_func,
                    &i2c_status);
        break;
    }

    case TRIGGERED:
        if (i2c_status != I2C_OK) {
            goto error;
        }

        // wait for conversion to complete
        os_setTimedCallback(job, os_getTime() + ms2osticks(DELAY_MEASURE_HPM), read_func);
        break;

    case READ: {
        i2c_xfer_ex(SHT3X_ADDRESS, sht3x.buf, 0, 6, I2C_TIMEOUT, job, read_func,
                    &i2c_status);
        break;
    }

    case DONE:
        if (i2c_status != I2C_OK) {
            goto error;
        }

        if (sht3x_check_crc(sht3x.buf, 2) && sht3x_check_crc(sht3x.buf + 3, 2)) {
            int32_t temp_ticks = (sht3x.buf[1] & 0xff) | ((int32_t)sht3x.buf[0] << 8);
            int32_t rh_ticks = (sht3x.buf[4] & 0xff) | ((int32_t)sht3x.buf[3] << 8);

            /**
            * formulas for conversion of the sensor signals, optimized for fixed point algebra:
            * Temperature       = 175 * S_T / 2^16 - 45
            * Relative Humidity = 100 * S_RH / 2^16
            */
            sht3x.pdata->temp = ((21875 * temp_ticks) >> 13) - 45000;
            sht3x.pdata->rh = ((12500 * rh_ticks) >> 13);
            // all done
            *sht3x.pstatus = eDevice_Ok;
        } else {
            debug_printf("CRC fail\r\n");
            *sht3x.pstatus = eDevice_Failed;
        }
        goto done;
    }
    state += 1;
    return;

error:
    *sht3x.pstatus = eDevice_Failed;
done:
    state = INIT;
    os_setCallback(job, sht3x.cb);
}

static const uint8_t CRC_POLYNOMIAL    = 0x31;
static const uint8_t CRC_INIT          = 0xff;

static inline bool sht3x_check_crc(uint8_t* data, uint16_t count) {
    return sht3x_generate_crc(data, count) == data[count];
}

static uint8_t sht3x_generate_crc(uint8_t* data, uint16_t count) {
    uint8_t crc = CRC_INIT;
    uint8_t current_byte;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

void sht3x_init(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus) {
    sht3x.cb = cb;
    sht3x.pstatus = pstatus;
    os_setCallback(job, init_func);
}

void sht3x_read(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus, sht3x_data* pdata) {
    sht3x.cb = cb;
    sht3x.pstatus = pstatus;
    sht3x.pdata = pdata;
    os_setCallback(job, read_func);
}
