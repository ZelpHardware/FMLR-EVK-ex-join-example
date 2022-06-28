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

#ifndef _DEVICE_TYPES_H_
#define _DEVICE_TYPES_H_

#include <stdint.h>
#include "lmic.h"

typedef enum {
    eDevice_None,
    eDevice_Initialized,
    eDevice_Ok,
    eDevice_Busy,
    eDevice_Timeout,
    eDevice_Failed
} eDeviceStatus_t;

#endif
