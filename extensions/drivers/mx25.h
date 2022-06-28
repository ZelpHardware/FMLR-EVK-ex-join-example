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

#ifndef _MX25_H_
#define _MX25_H_

#include "driver_types.h"

#define MX25_FLASH_SIZE     0x80000 // 512KB

#define MX25_INIT_STEPS     MX25_INIT, MX25_DONE

// Public flash functions
eDeviceStatus_t mx25_cmd_RDSR(u1_t*);
eDeviceStatus_t mx25_cmd_RDID(u4_t* id);
eDeviceStatus_t mx25_cmd_DP();
eDeviceStatus_t mx25_cmd_PP(uint32_t flash_address, uint8_t* source_address, uint32_t byte_length);
eDeviceStatus_t mx25_cmd_SE(uint32_t flash_address);
eDeviceStatus_t mx25_cmd_READ(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length);

void mx25_init(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus);
void mx25_init_no_dp(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus);

#endif
