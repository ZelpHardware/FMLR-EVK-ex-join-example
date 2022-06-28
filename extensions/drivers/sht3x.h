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

#ifndef _SHT3X_H_
#define _SHT3X_H_

#include "driver_types.h"

#define SHT3X_INIT_STEPS   SHT3X_INIT, SHT3X_DONE

typedef struct {
    int32_t temp;
    int32_t rh;
} sht3x_data;

void sht3x_init(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus);

void sht3x_read(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus, sht3x_data* pdata);

#endif
