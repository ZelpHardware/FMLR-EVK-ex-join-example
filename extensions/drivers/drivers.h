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

#ifndef _DRIVERS_H_
#define _DRIVERS_H_

#ifdef DRIVER_battery
#include "battery.h"
#endif

#ifdef DRIVER_sht3x
#include "sht3x.h"
#endif

#ifdef DRIVER_sht21
#include "sht21.h"
#endif

#ifdef DRIVER_mx25
#include "mx25.h"
#endif

#endif
