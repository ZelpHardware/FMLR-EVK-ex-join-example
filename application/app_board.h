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

// to be included from board.h

// -------------------------------------------
#if defined(CFG_fmlr_72_x_stl0_board)

#define GPIO_LEDR	BRD_GPIO_EX(PORT_H, 0, BRD_GPIO_ACTIVE_LOW) // red
#define GPIO_LEDG	BRD_GPIO_EX(PORT_H, 1, BRD_GPIO_ACTIVE_LOW) // green
#define GPIO_LEDB	BRD_GPIO_EX(PORT_C, 1, BRD_GPIO_ACTIVE_LOW) // blue

#define GPIO_DBG_LED	GPIO_LEDR

#define GPIO_DBG_TX		BRD_GPIO_AF(PORT_A, 9, 4)
#define GPIO_DBG_RX     BRD_GPIO_AF(PORT_A, 10, 4)
#define BRD_DBG_UART	1

#define BRD_I2C         1
#define GPIO_I2C_SCL  BRD_GPIO_AF(PORT_B, 6, 1)
#define GPIO_I2C_SDA  BRD_GPIO_AF(PORT_B, 9, 4)

#define GPIO_BUTTON_DEVBOARD BRD_GPIO(PORT_A, 0) //

#elif defined(CFG_fmlr_61_x_ma625)

// Using P0.0 as RX, P0.1 as TX (IO mapping A)
#define BRD_DBG_UART	0
#define GPIO_BUTTON_DEVBOARD BRD_GPIO(3, 7)
#define GPIO_LEDR BRD_GPIO(3,4)
#define GPIO_LEDG BRD_GPIO(4,2)
#define GPIO_LEDB BRD_GPIO(3,5)

#endif
