/*    __  __ _____ _____   ____  __  __ _____ _____ ____
 *   |  \/  |_   _|  __ \ / __ \|  \/  |_   _/ ____/ __ \
 *   | \  / | | | | |__) | |  | | \  / | | || |   | |  | |
 *   | |\/| | | | |  _  /| |  | | |\/| | | || |   | |  | |
 *   | |  | |_| |_| | \ \| |__| | |  | |_| || |___| |__| |
 *   |_|  |_|_____|_|  \_\\____/|_|  |_|_____\_____\____/
 *
 * Copyright (c) 2020 Miromico AG
 * All rights reserved.
 */

// to be included from board.h

#define GPIO_RST        BRD_GPIO(0, 3)      // reset n:             port 0, pin 3
#define GPIO_DIO1       BRD_GPIO(4, 6)      // DIO1 (interrupt):    port 4, pin 6
#define GPIO_BUSY       BRD_GPIO(4, 7)      // RF busy pin:         port 4, pin 7
#define GPIO_TXRX_EN    BRD_GPIO(4, 5)      // RF switch vdd:       port 4, pin 5

#define GPIO_LED1       BRD_GPIO(4, 4)      // red on module

#define BRD_sx1261_radio

#define BRD_RADIO_SPI   0
#define GPIO_NSS        BRD_GPIO(0, 7)      // SPI0M NSS:           port 0, pin 7
#define GPIO_SCK        BRD_GPIO(0, 4)      // SPI0M SCK:           port 0, pin 4
#define GPIO_MISO       BRD_GPIO(0, 6)      // SPI0M MISO:          port 0, pin 6
#define GPIO_MOSI       BRD_GPIO(0, 5)      // SPI0M MOSI:          port 0, pin 5

#define GPIO_DBG_LED    GPIO_LED1           // Debug Led

// power consumption
#ifndef BRD_PWR_RUN_UA
#define BRD_PWR_RUN_UA 6000
#endif

#ifndef BRD_PWR_S0_UA
#define BRD_PWR_S0_UA  2000
#endif

#ifndef BRD_PWR_S1_UA
#define BRD_PWR_S1_UA  12
#endif

#ifndef BRD_PWR_S2_UA
#define BRD_PWR_S2_UA  5
#endif

// brown-out
#define BRD_borlevel   9 // RM0376, pg 116: BOR level 2, around 2.0 V
