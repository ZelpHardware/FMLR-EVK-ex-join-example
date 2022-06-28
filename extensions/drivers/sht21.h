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

#ifndef _SHT21_H_
#define _SHT21_H_

#include "driver_types.h"

#define SHT21_INIT_STEPS   SHT21_INIT, SHT21_DONE

typedef struct {
    int16_t temp; // T in 1/100 °C
    uint8_t rh;   // rH in 0.5 %
} sht21_data;

void sht21_init(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus);

void sht21_read(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus, sht21_data* pdata);

#endif
