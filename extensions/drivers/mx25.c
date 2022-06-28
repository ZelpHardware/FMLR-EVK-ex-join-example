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

#include "mx25.h"

static struct {
    osjobcb_t cb;

    u1_t** buffer;
    u2_t  size;

    bool deepSleep;
    bool initDeepSleep;

    eDeviceStatus_t* pstatus;
} mx25;

/*** MX25 series command hex code definition ***/
#define    FlashID          0xc22813

// ID commands
#define    FLASH_CMD_RDID      0x9F    //RDID (Read Identification)

// Register commands
#define    FLASH_CMD_RDSR      0x05    //RDSR (Read Status Register)

//Mode setting command(s)
#define    FLASH_CMD_DP       0xB9    //DP (Deep Power Down)

//Erase comands
#define    FLASH_CMD_SE       0x20    //SE (Sector Erase)
#define    FLASH_CMD_BE32K    0x52    //BE32K (Block Erase 32kb)
#define    FLASH_CMD_BE       0xD8    //BE (Block Erase)
#define    FLASH_CMD_CE       0x60    //CE (Chip Erase) hex code: 60 or C7

// Read command(s)
#define    FLASH_CMD_READ        0x03    //READ (1 x I/O)

// Program command(s)
#define    FLASH_CMD_WREN     0x06    //WREN (Write Enable)
#define    FLASH_CMD_PP       0x02    //PP (page program)

/*** MX25 series timing definition ***/
#define    tPUW             ms2osticks(10)  // 10ms
#define    tCRDP            us2osticks(1)   // 1 us
#define    tRDP             us2osticks(35)  // 35 us
#define    tPP              ms2osticks(10)  // 10ms
#define    tSE              ms2osticks(240) // 240ms

// Flash control register mask define
// status register
#define    FLASH_WIP_MASK         0x01

static inline void send_addr(uint32_t flash_address) {
    spi_xfer((flash_address >> 16));
    spi_xfer((flash_address >> 8));
    spi_xfer((flash_address));
}

static bool is_busy(void) {
    uint8_t reg;

    mx25_cmd_RDSR(&reg);
    return ((reg & FLASH_WIP_MASK) == FLASH_WIP_MASK);
}

static eDeviceStatus_t mx25_cmd_WREN() {
    // device will not react to command in deep sleep
    if (mx25.deepSleep) { return eDevice_Busy; }

    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    // Write Enable command = 0x06, Setting Write Enable Latch Bit
    spi_xfer(FLASH_CMD_WREN);

    // Chip select inactive to end a flash command
    spi_cs_io(GPIO_FLASH_NSS, 0);

    return eDevice_Ok;
}

eDeviceStatus_t mx25_cmd_RDSR(u1_t* reg) {
    // device will not react to command in deep sleep
    if (mx25.deepSleep) { return eDevice_Busy; }

    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    // Send command
    spi_xfer(FLASH_CMD_RDSR);
    // read register
    *reg = spi_xfer(0);

    // Chip select inactive to end a flash command
    spi_cs_io(GPIO_FLASH_NSS, 0);

    return eDevice_Ok;
}

eDeviceStatus_t mx25_cmd_RDID(u4_t* id) {
    // device will not react to command in deep sleep
    if (mx25.deepSleep) { return eDevice_Busy; }
    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    // Send command
    spi_xfer(FLASH_CMD_RDID);

    // Get manufacturer identification, device identification
    *id = (u4_t) spi_xfer(0) << 16;
    *id |= (u4_t) spi_xfer(0) << 8;
    *id |= spi_xfer(0);

    // Chip select inactive to end a command
    spi_cs_io(GPIO_FLASH_NSS, 0);

    return eDevice_Ok;
}

eDeviceStatus_t mx25_cmd_DP() {
    // already sleeping?
    if (mx25.deepSleep) { return eDevice_Ok; }

    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    // Deep Power Down Mode command
    spi_xfer(FLASH_CMD_DP);

    // Chip select inactive to end a flash command
    spi_cs_io(GPIO_FLASH_NSS, 0);
    mx25.deepSleep = true;

    return eDevice_Ok;
}

eDeviceStatus_t mx25_cmd_PP(uint32_t flash_address, uint8_t* source_address, uint32_t byte_length) {
    uint32_t index;

    // device will not react to command in deep sleep
    if (mx25.deepSleep) { return eDevice_Busy; }

    // Check flash address
    if (flash_address >= MX25_FLASH_SIZE) { return eDevice_Failed; }

    // Check flash is busy or not
    if (is_busy()) { return eDevice_Busy; }

    // Setting Write Enable Latch bit
    mx25_cmd_WREN();

    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    // Write Page Program command
    spi_xfer(FLASH_CMD_PP);
    send_addr(flash_address);

    // Set a loop to download whole page data into flash's buffer
    // Note: only last 256 byte will be programmed
    for (index = 0; index < byte_length; index++) {
        spi_xfer(*(source_address + index));
    }

    // Chip select inactive to end a flash command
    spi_cs_io(GPIO_FLASH_NSS, 0);

    return eDevice_Ok;
}

eDeviceStatus_t mx25_cmd_SE(uint32_t flash_address) {
    // device will not react to command in deep sleep
    if (mx25.deepSleep) { return eDevice_Busy; }

    // Check flash address
    if (flash_address >= MX25_FLASH_SIZE) { return eDevice_Failed; }

    // Check flash is busy or not
    if (is_busy()) { return eDevice_Busy; }

    // Setting Write Enable Latch bit
    mx25_cmd_WREN();

    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    //Write Sector Erase command = 0x20;
    spi_xfer(FLASH_CMD_SE);
    send_addr(flash_address);

    // Chip select inactive to end a flash command
    spi_cs_io(GPIO_FLASH_NSS, 0);

    return eDevice_Ok;
}

eDeviceStatus_t mx25_cmd_READ(uint32_t flash_address, uint8_t* target_address, uint32_t byte_length) {
    uint32_t index;

    // device will not react to command in deep sleep
    if (mx25.deepSleep) { return eDevice_Busy; }

    // Check flash address
    if (flash_address >= MX25_FLASH_SIZE) { return eDevice_Failed; }

    // Chip select active to start a flash command
    spi_cs_io(GPIO_FLASH_NSS, 1);

    // Write READ command and address
    spi_xfer(FLASH_CMD_READ);
    send_addr(flash_address);

    // Set a loop to read data into buffer
    for (index = 0; index < byte_length; index++) {
        // Read data one byte at a time
        *(target_address + index) = spi_xfer(0);
    }

    // Chip select inactive to end a flash command
    spi_cs_io(GPIO_FLASH_NSS, 0);

    return eDevice_Ok;
}

static void init_func(osjob_t* job) {
    static enum {
        INIT, LO, HI, ID, SLEEP
    } state;

    switch (state) {
    default:
    case INIT:
        // init spi for mx25
        spi_init(0, 0, 0);
        // Initialize CS pin
        spi_cs_io(GPIO_FLASH_NSS, 0);
        // power up delay
        os_setTimedCallback(job, os_getTime() + tPUW, init_func);
        break;

    // if device is already in deep power down, it needs a CS toggle
    // to go back to standby, otherwise RDID command will be ignored
    case LO:
        spi_cs_io(GPIO_FLASH_NSS, 1);
        os_setTimedCallback(job, os_getTime() + tCRDP, init_func);
        break;
    case HI:
        spi_cs_io(GPIO_FLASH_NSS, 0);
        os_setTimedCallback(job, os_getTime() + tRDP, init_func);
        mx25.deepSleep = false;
        break;

    case ID: {
        u4_t flash_id = 0;
        mx25_cmd_RDID(&flash_id);
        if (flash_id != FlashID) {
            debug_printf("FlashID mismatch (0x%02x)\r\n", flash_id);
            goto error;
        }
        if (mx25.initDeepSleep) {
            os_setCallback(job, init_func);
            break;
        } else {
            // keep device awake
            debug_printf("mx25 no DP!!\r\n");
            *mx25.pstatus = eDevice_Initialized;
            goto done;
        }
    }

    case SLEEP: {
        mx25_cmd_DP();
        *mx25.pstatus = eDevice_Initialized;
        goto done;
    }
    }
    state += 1;
    return;

error:
    *mx25.pstatus = eDevice_Failed;
done:
    state = INIT;
    os_setCallback(job, mx25.cb);
}

void mx25_init(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus) {
    mx25.cb = cb;
    mx25.pstatus = pstatus;
    mx25.deepSleep = true;
    mx25.size = 0;
    mx25.buffer = 0;
    mx25.initDeepSleep = true;
    os_setCallback(job, init_func);
}

void mx25_init_no_dp(osjob_t* job, osjobcb_t cb, eDeviceStatus_t* pstatus) {
    mx25.cb = cb;
    mx25.pstatus = pstatus;
    mx25.deepSleep = true;
    mx25.size = 0;
    mx25.buffer = 0;
    mx25.initDeepSleep = false;
    os_setCallback(job, init_func);
}
