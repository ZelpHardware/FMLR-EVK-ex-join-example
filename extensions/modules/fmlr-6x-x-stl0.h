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

#if defined(CFG_fmlr_61_x_stl0_board)
#define BRD_sx1261_radio
#elif defined(CFG_fmlr_62_x_stl0_board)
#define BRD_sx1262_radio
#endif

#define GPIO_RST    BRD_GPIO(PORT_B, 11)

#define BRD_sx1261_radio

#define GPIO_DIO1   BRD_GPIO(PORT_A, 1)
#define GPIO_BUSY   BRD_GPIO(PORT_C, 4)

#define GPIO_TXRX_EN    BRD_GPIO(PORT_B, 12)
#define BRD_RADIO_SPI   2
#define GPIO_NSS    BRD_GPIO(PORT_A, 4)
#define GPIO_SCK    BRD_GPIO_AF(PORT_B, 10, 5)
#define GPIO_MISO   BRD_GPIO_AF(PORT_B, 14, 0)
#define GPIO_MOSI   BRD_GPIO_AF(PORT_B, 15, 0)

#define BRD_SPI 1
#define GPIO_FLASH_NSS  BRD_GPIO(PORT_B, 13)
#define GPIO_BRD_SCK    BRD_GPIO_AF(PORT_B, 3, 0)
#define GPIO_BRD_MISO   BRD_GPIO_AF(PORT_B, 4, 0)
#define GPIO_BRD_MOSI   BRD_GPIO_AF(PORT_B, 5, 0)

#define GPIO_LED1   BRD_GPIO_EX(PORT_B, 8, BRD_GPIO_ACTIVE_LOW) // on board LED (might not be mounted)

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
