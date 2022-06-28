// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "lmic.h"
#include "peripherals.h"

#include "board.h"
#include "bootloader.h"
#include "boottab.h"
#include "gpio.h"
#include "uart.h"
#include "spim.h"
#include "ioman.h"
#include "system_max32625.h"
#include "clkman.h"
#include "tmr.h"
#include "nvic_table.h"
#include "rtc.h"
#include "lp.h"
#include "prng.h"
#include "wdt2.h"
#include "usb_regs.h"
#include "adc_regs.h"
#include "flc_regs.h"

#if defined(CFG_perso)
#include "perso.h"
#endif

#if defined(SVC_eefs)
#include "eefs/eefs.h"
#endif

#if defined(SVC_pwrman)
#include "pwrman/pwrman.h"
#endif

// HAL state
static struct {
    s4_t irqlevel;
    u4_t ticks;
    int watchcount;
    u4_t reset;
#ifdef CFG_rtstats
    struct {
        uint32_t run;                   // ticks running
        uint32_t sleep[HAL_SLEEP_CNT];  // ticks sleeping
    } rtstats;
#endif
    u1_t maxsleep[HAL_SLEEP_CNT-1]; // deep sleep restrictions
    u1_t battlevel;
    boot_boottab* boottab;
} HAL;

uint32_t SystemCoreClock;            /*!< System Clock Frequency (Core Clock) */

#ifdef CFG_DEBUG
#define SLEEP_LP1_OVERHEAD (5000) //WITH UART debug output.  4000 os_ticks = 64ms
#define SLEEP_WFI_OVERHEAD (8)
#else
#define SLEEP_LP1_OVERHEAD (16)    //WITHOUT UART debug output   8 os ticks = 244us
#define SLEEP_WFI_OVERHEAD (8)    //WITHOIU UART debug output   4 os_ticks = 122us
#endif
#define WD_OVERHEAD_FOR_SLEEP (sec2osxticks(1))

#if BRD_DBG_UART == 0
#define DBG_USART MXC_UART0
#elif BRD_DBG_UART == 1
#define DBG_USART MXC_UART1
#elif BRD_DBG_UART == 2
#define DBG_USART USART2
#elif BRD_DBG_UART == 4
#define DBG_USART USART4
#else
#error "Unsupported value for BRD_RADIO_SPI"
#endif

#define DBG_UART_BR 115200
//#define max_printf(f,...)  do { } while (0)
#define max_printf  debug_printf

// -----------------------------------------------------------------------------
// Panic

// don't change these values, so we know what they are in the field...
enum {
    PANIC_HAL_FAILED    = 0,
    PANIC_LSE_NOSTART   = 1,
    PANIC_CAL_FAILED    = 2,
    PANIC_STS_FAILED    = 3,
    PANIC_SR_BUSY       = 4,
};

#ifdef CFG_panic911
__attribute__((noinline)) // ensure function has a frame, since called from naked
static void call911 (uint32_t reason, uint32_t addr) {
    struct {
        uint32_t magic;
        unsigned char deveui[8];
        uint32_t reason;
        uint32_t addr;
    } info;
    info.magic = 0x504c4548; // HELP
    memcpy(info.deveui, BOOT_DEVINFO->deveui, 8);
    info.reason = reason;
    info.addr = addr;
    memcpy(LMIC.frame, &info, LMIC.dataLen = sizeof(info));

#if defined(CFG_eu868)
    LMIC.freq = 868300000;
    LMIC.rps = updr2rps(DR_SF12);
#elif defined(CFG_us915)
    LMIC.freq = 903700000;
    LMIC.rps = updr2rps(DR_SF10);
#else
#error "Unsupported region"
#endif
    LMIC.txpow = 14;
    os_radio(RADIO_TX);
}
#endif

__attribute__((noreturn))
static void panic (uint32_t reason, uint32_t addr) {
    // disable interrupts
    __disable_irq();

#ifdef CFG_panic911
    // yelp for help
    call911(reason, addr);
#endif

    // call bootloader's panic function
    HAL.boottab->panic(reason, addr);
    // not reached
}

__attribute__((noreturn, naked))
void hal_failed () {
    // get return address
    uint32_t addr;
    __asm__("mov %[addr], lr" : [addr]"=r" (addr) : : );
    // in thumb mode the linked address is the address of the calling instruction plus 4 bytes
    addr -= 4;

#ifdef SVC_backtrace
    // log address of assertion
    bt_addr(__LINE__, addr);
    // save trace to EEPROM
    bt_save();
#endif

    // call panic function
    panic(PANIC_HAL_FAILED, addr);
    // not reached
}


// -----------------------------------------------------------------------------
// timer rollover based watchdog

void hal_watchcount (int cnt) {
    HAL.watchcount = cnt;
}

// -----------------------------------------------------------------------------
// SPI

#if BRD_RADIO_SPI == 0
#define SPIx                    MXC_SPIM0
#elif BRD_RADIO_SPI == 1
#define SPIx                    MXC_SPIM1
#elif BRD_RADIO_SPI == 2
#define SPIx                    MXC_SPIM2
#else
#error "Unsupported value for BRD_RADIO_SPI"
#endif

static void hal_spi_init () {

		uint32_t err;
		spim_cfg_t spim_cfg;
		sys_cfg_spim_t spim_sys_cfg;

		spim_cfg.mode = 0;
		spim_cfg.ssel_pol = 0;
		spim_cfg.baud = 10000000; //10MHz. SX1261 max is 16MHz. MAX max is 48MHz


		spim_sys_cfg.io_cfg = (ioman_cfg_t)IOMAN_SPIM0(1,0,0,0,0,0,0,1); //This config doesn't use NSS. (io, ss0, ss1, ss2, ss3, ss4, q, f)
		spim_sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;

		err = SPIM_Init(SPIx, &spim_cfg, &spim_sys_cfg);
		ASSERT(err==0);

    CFG_PIN(GPIO_NSS, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);

    // configure I/O lines and disable clock
    hal_spi_select(0);
}

void hal_spi_select (int on) {

  if (on) {
    SET_PIN(GPIO_NSS, 0);
  }
  else {
    SET_PIN(GPIO_NSS, 1);
  }

//#define SHUTDOWN_SPI_IF_NOT_SELECTED
#ifdef SHUTDOWN_SPI_IF_NOT_SELECTED //This is a try to save power, but can't be verified at this point in time.
    //debug_printf("SPI SHUTDOWN_SPI_IF_NOT_SELECTED\r\n");
    uint32_t err;
    spim_cfg_t spim_cfg;
    sys_cfg_spim_t spim_sys_cfg;

    spim_cfg.mode = 0;
    spim_cfg.ssel_pol = 0;
    spim_cfg.baud = 10000000; //10MHz. SX1261 max is 16MHz. MAX max is 48MHz


    spim_sys_cfg.io_cfg = (ioman_cfg_t)IOMAN_SPIM0(1,0,0,0,0,0,0,1); //This config doesn't use NSS. (io, ss0, ss1, ss2, ss3, ss4, q, f)
    spim_sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
    if (on) {
        err = SYS_SPIM_Init(MXC_SPIM0, &spim_cfg, &spim_sys_cfg);
        ASSERT(err==0);
        SET_PIN(GPIO_NSS, 0);
    }
    else {
        err = SYS_SPIM_Shutdown(MXC_SPIM0);
        ASSERT(err==0);
        SET_PIN(GPIO_NSS, 1);
    }
#endif

}


// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    //debug_printf("SPI out: %x      ", (u1_t)(out));
    uint8_t read_buffer[1];
    uint8_t output[1];
    output[0] = out;
    spim_req_t req;
    req.ssel = 0;
    req.deass = 1;
    req.tx_data = output;
    req.rx_data = read_buffer;
    req.width = SPIM_WIDTH_1;
    req.len = 1;
    req.read_num = 1;
    req.callback = NULL;

    SPIM_Trans(MXC_SPIM0, &req);
    //debug_printf("in: %x\r\n", (u1_t)(read_buffer[0]));

    return read_buffer[0];
}


// -----------------------------------------------------------------------------
// Clock and Time
//
// R0 is the main run mode. R1-2 are only used on the way to and from their
// respective sleep modes. Also, timer roll-over is handled in the R mode
// corresponding to the current sleep mode to avoid waking up completely.
// Otherwise, a wake-up out of any sleep mode will return from the
// corresponding R mode back to R0.
//
// S0 is a low-latency sleep mode, i.e. it can be entered and exited in less
// than 1 tick.
//
// Run modes:
// - R0: Run       32MHz HSI  ~ ?   mA
// - R1: Run        4MHz MSI  ~ ?   mA
// - R2: LP Run    65kHz MSI  ~ ?   mA     - flash off (must run from RAM)
//
// Sleep modes: (to be measured)
// - S0: Sleep     32MHz HSI  ~ ?   mA     - e.g. when high-speed-clock-dependent peripherals are active
// - S1: Sleep      4MHz MSI  ~ ?   mA     - e.g. when the time is near
// - S2: Stop            n/a  ~ ?   mA     - all other cases
//
// Peripherals and applications can use the hal_setMaxSleep() and
// hal_clearMaxSleep() APIs in matching pairs to restrict the HAL from entering
// the deeper sleep modes.
//
// Wake-up times to R0 (empirical) (to be measured)
// - from S0         < ? tick
// - from S1         ~ ? ticks
// - from S2         ~ ? ticks


// Busy wait on condition with timeout (about 10s)
#define SAFE_while(reason, expr) do { \
    uint32_t __timeout = (1 << 25); \
    while( expr ) { \
        if( __timeout-- == 0 ) { \
            panic(reason, hal_getpc()); \
        } \
    } \
} while (0)

// RTC timer 0 interrupt handler (time_irq)
static void RTC0_handler_Compare0(){
    uint32_t count = RTC_GetCount();
    uint32_t nextCompareCount = count | ((uint32_t)(0x1FFF));
    HAL.ticks += 1;
    RTC_ClearFlags(MXC_F_RTC_FLAGS_COMP0);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    RTC_SetCompare(0, nextCompareCount);

#if CFG_watchdog
    WDT2_Reset();
#endif
}

// initialize the LSE and the LPTIM1 peripheral
static void time_init (void) {
    // RTC setup for Compare timer 0
    uint32_t err;
    rtc_cfg_t RTCconfig;
    RTCconfig.compareCount[0] = 0x1FFF;
    RTCconfig.compareCount[1] = 0;
    RTCconfig.prescaler = RTC_PRESCALE_DIV_2_0; //4096Hz clock
    RTCconfig.prescalerMask = RTC_PRESCALE_DIV_2_0; //must be smaller or equal to prescaler setting
    RTCconfig.snoozeCount = 0;
    RTCconfig.snoozeMode = RTC_SNOOZE_DISABLE; //not configured
    err = RTC_Init(&RTCconfig);
    ASSERT(err==E_NO_ERROR);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);
    RTC_EnableINT(MXC_F_RTC_INTEN_COMP0);
    RTC_Start();

#if CFG_watchdog
    debug_printf("WatchDog enabled\r\n");
    WDT2_Init(0, MXC_V_WDT2_UNLOCK_KEY);
    WDT2_EnableReset(WDT2_PERIOD_2_15_CLKS, MXC_V_WDT2_UNLOCK_KEY);  // 2^15/8KHz = 4.096s
    WDT2_Start(MXC_V_WDT2_UNLOCK_KEY);
#endif
}


// stub from BasicsMac. Not used for MAX32625
__attribute__((always_inline)) static inline void flash_off (void) {
}

__attribute__((always_inline)) static inline void flash_on (void) {
}

typedef enum {
    SLEEP_INTERRUPT_REASON_T_RTC_COMP0,
    SLEEP_INTERRUPT_REASON_T_RTC_OVERFLOW,
    SLEEP_INTERRUPT_REASON_T_OTHER,
} SLEEP_INTERRUPT_REASON_T;

SLEEP_INTERRUPT_REASON_T sleep2(uint32_t nextCompareCount) {
#ifdef CFG_DEBUG
    while(UART_PrepForSleep(MXC_UART0) != 0); //flush UART
    UART_Shutdown(MXC_UART0);
    //MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_FIRMWARE_RESET;
#endif
    RTC_ClearFlags(MXC_F_RTC_FLAGS_COMP0|MXC_F_RTC_FLAGS_COMP1|MXC_F_RTC_FLAGS_OVERFLOW|MXC_F_RTC_FLAGS_PRESCALE_COMP); // clear all RTC flags
    RTC_SetCompare(0, nextCompareCount);
    LP_ClearWakeUpConfig();
    LP_ClearWakeUpFlags();

    //Configure all enabled GPIO interrupts as wake-up
    for(uint32_t port=0; port<NB_OF_PORTS; port++){
        for(uint32_t pin=0; pin<8;pin++){
            if(MXC_GPIO->inten[port] & (1<<pin)){
                gpio_cfg_t gpio = {0, 0, GPIO_FUNC_GPIO, GPIO_PAD_INPUT};
                gpio.port = port;
                gpio.mask = 1<<pin;
                uint32_t int_mode = MXC_GPIO->int_mode[port];
                int_mode = int_mode >> (pin*4);
                int_mode &= 0xF;
                uint32_t act_high = 1;
                lp_pu_pd_select_t pull = LP_WEAK_PULL_DOWN;
                if((int_mode == GPIO_INT_FALLING_EDGE) || (int_mode == GPIO_INT_LOW_LEVEL)){
                    act_high = 0;
                    pull = LP_WEAK_PULL_UP;
                }
                //debug_printf("%x,%x,%x,%x,", gpio.port, gpio.mask, act_high, pull);
                uint32_t err = LP_ConfigGPIOWakeUpDetect(&gpio, act_high, pull);
                MXC_ASSERT(err==0);
            }
        }
    }

    LP_ConfigRTCWakeUp(1, 0, 0, 1); //wake-up on RTC Compare 0 and RTC counter roll-over
#ifdef CFG_DEBUG
    //while(UART_PrepForSleep(MXC_UART_GET_UART(0)) != 0); //flush UART

    //Shutdown UART
    //uint32_t err = UART_Shutdown(MXC_UART0);
    //ASSERT(err==0);
    //SPIM_Shutdown(MXC_SPIM0);
#endif
    LP_EnterLP1(); //sleep

#ifdef CFG_DEBUG
    const uart_cfg_t uart_cfg = {
        .parity = UART_PARITY_DISABLE,
        .size = UART_DATA_SIZE_8_BITS,
        .extra_stop = 0,
        .cts = 0,
        .rts = 0,
        .baud = DBG_UART_BR,
    };

    const sys_cfg_uart_t uart_sys_cfg = {
        .clk_scale = CLKMAN_SCALE_AUTO,
        .io_cfg = IOMAN_UART(BRD_DBG_UART, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0)
    };

    uint32_t err = UART_Init(MXC_UART0, &uart_cfg, &uart_sys_cfg);
    ASSERT(err==0);
#endif

    // read and check wake-up flags after wake-up
    uint32_t irq = LP_GetWakeUpFlags();
    uint32_t wud0 = MXC_PWRMAN->wud_seen0;
    uint32_t wud1 = MXC_PWRMAN->wud_seen1;

    // clear the GPIO irq pending flags of ones they were not responsible for the wake-up.
    // rationale: after wake-up from deep-sleep the intfl is set for all configured wake-up pins.
    for(uint32_t port=0; port<NB_OF_PORTS; port++){
        for(uint32_t pin=0; pin<8;pin++){
            if(MXC_GPIO->inten[port] & (1<<pin)){
                if((port < 4) && (((wud0) & (1 << (8*port + pin))) == 0)){
                    MXC_GPIO->intfl[port] = 1 << pin;
                }
                else if((port==4) && ((wud1 & (1 << pin)) == 0)){
                    MXC_GPIO->intfl[port] = 1 << pin;
                }
            }
        }
    }

    //debug_printf("irq:%x\r\n", irq); //=> ok
    //debug_printf("wud: %x %x\r\n", wud0, wud1); //=> ok

    if(irq & MXC_F_PWRSEQ_FLAGS_RTC_CMPR0){
        return SLEEP_INTERRUPT_REASON_T_RTC_COMP0;
    }
    else if(irq & MXC_F_PWRSEQ_FLAGS_RTC_ROLLOVER){
        return SLEEP_INTERRUPT_REASON_T_RTC_OVERFLOW;
    }

    //debug_printf("irq:%x\r\n", irq); //=> ok
    //debug_printf("wud: %x %x\r\n", MXC_PWRMAN->wud_seen1, MXC_PWRMAN->wud_seen0); //=> ok

    LP_ClearWakeUpFlags();

    return SLEEP_INTERRUPT_REASON_T_OTHER;
}


SLEEP_INTERRUPT_REASON_T sleep1(uint32_t nextCompareCount) {
    RTC_ClearFlags(MXC_F_RTC_FLAGS_COMP0|MXC_F_RTC_FLAGS_COMP1|MXC_F_RTC_FLAGS_OVERFLOW|MXC_F_RTC_FLAGS_PRESCALE_COMP); // clear all RTC TMR flags
    RTC_SetCompare(0, nextCompareCount);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);

    //while(UART_PrepForSleep(MXC_UART_GET_UART(0)) != 0); //flush UART
    //processor sleep
    __WFI();

    // read and check ISR after wake-up
    uint32_t irq = RTC_GetFlags();

    if(irq & MXC_F_RTC_FLAGS_COMP0){
        return SLEEP_INTERRUPT_REASON_T_RTC_COMP0;
    }

    else if (irq & MXC_F_RTC_FLAGS_OVERFLOW){
        return SLEEP_INTERRUPT_REASON_T_RTC_OVERFLOW;
    }
    return SLEEP_INTERRUPT_REASON_T_OTHER;
}

SLEEP_INTERRUPT_REASON_T sleep0(uint32_t nextCompareCount) {
    return sleep1(nextCompareCount);
}

static const SLEEP_INTERRUPT_REASON_T(*sleepfuncs[])(uint32_t) = {
    sleep0,
    sleep1,
    sleep2,
};

// LP1 sleep: MAX32625 is controlled by the Power Sequencer, running on the RTC. The CPU is in deep sleep.
// All clocks are gated off and almost all digital logic is in a static, low-power state.
// SRAM and register content is preserved.
// Note: Wake-up only from RTC or GPIO
// Note: It is recommended to turn off all unused analog circuitry
//
//
//                          roll-over every 272 years               roll-over every 2 sec     32768Hz clock (divided by 2)
//                                   |                                       |                   |
//                                   |                 HAL.ticks (htt)       |    os_ticks (ltt) |
//                                   V ____________________A________________ V ________A________ V
//                                    |                                     | |                 |
// os_ticks                           XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX
//                                                                                            <->          3 bit-shift
//                                                                            <----ltt_rtc--->          16-3 bit-shift
//                                                     <--------htt_rtc------>                     32-(16-3) bit-shift
// RTC         XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX
//             |__________________ __________________| |__________________ __________________|
//                                V                   A                   V
//                            rtc_rollover_counter    |       rtc_count (nextCompareCount)
//                                                    |
//                                           roll-over every 12 days
//
//Time measurements: sleep_htt_ltt_LP1: 15us < os_tick (30.5us).
//                   Flushing UART (2.5ms) and additional debug printfs (20ms)
//                   =>
// Before this function is called, make sure the target time (htt/ltt) encounters for execution time overhead of this function.

static void sleep (int stype, u4_t htt, u4_t ltt) {
    uint32_t rtc_count;
    uint32_t ltt_rtc = (ltt >> 3) & (0x1FFF);
    uint32_t htt_rtc = (htt << (16-3)) & (~(0x1FFF));
    uint32_t rtc_rollover_counter = htt >> (32-(16-3));
    uint32_t nextCompareCount = htt_rtc | ltt_rtc;
    SLEEP_INTERRUPT_REASON_T irq_reason=0;

    u8_t xnow = hal_xticks();
    u8_t xtarget = htt;
    xtarget <<= 16;
    xtarget |= ltt;
    s8_t xdelta = xtarget - xnow;

    //exit without further calculations, which might take more time than available
    if(xdelta < SLEEP_WFI_OVERHEAD){
        return;
    }

#if CFG_watchdog
    if(xdelta > WD_OVERHEAD_FOR_SLEEP) {
        WDT2_DisableReset(MXC_V_WDT2_UNLOCK_KEY);
    }
#endif
    //debug use
    //rtc_count = RTC_GetCount();
    //debug_printf("sleep WFI...ca %dh, %dms, ltt:%x\r\n", (htt-hticks)/1800, (nextCompareCount-rtc_count)/4, ltt);
    //debug_printf("sleep_htt_ltt       hticks:%x     htt:%x,     ltt:%x\r\n", hticks, htt, ltt);
    //debug_printf("sleep rtc_rollover_counter:%x htt_rtc:%x, ltt_rtc:%x\r\n", rtc_rollover_counter, htt_rtc, ltt_rtc);
    //debug_printf("RTC_GetCount(): %x, nextCompareCount:%x\r\n", rtc_count, nextCompareCount);

//#ifdef CFG_DEBUG
    rtc_count = RTC_GetCount();
//    if((nextCompareCount - rtc_count)/4 > 30) { //only debug_printf if we have more than 30ms of time
//      debug_printf("sleep type: %x ...ca %dh, %dms\r\n", stype, (htt-HAL.ticks)/1800, (nextCompareCount-rtc_count)/4);
//    }
//#endif

    do {
        //call sleep function
        irq_reason = sleepfuncs[stype](nextCompareCount);
            rtc_count = RTC_GetCount();

        //administration of HAL.ticks
            HAL.ticks = (rtc_rollover_counter << (32-(16-3))) | rtc_count >> (16-3);

            if (irq_reason == SLEEP_INTERRUPT_REASON_T_OTHER) {
                max_printf("g'day NVIC->ISPR[0]:%x\r\n", NVIC->ISPR[0]); // other interrupt, g'day
        break;
            }
            else if (irq_reason == SLEEP_INTERRUPT_REASON_T_RTC_COMP0) {
        NVIC_ClearPendingIRQ(RTC0_IRQn);
        break;
    }
        else if (irq_reason == SLEEP_INTERRUPT_REASON_T_RTC_OVERFLOW){
//            max_printf("roll-over\r\n");
            rtc_rollover_counter++;
        }
  } while (HAL.ticks<htt);

    //We slept. So enable COMP0 for os_ticks again
    RTC_ClearFlags(MXC_F_RTC_FLAGS_COMP0|MXC_F_RTC_FLAGS_COMP1|MXC_F_RTC_FLAGS_OVERFLOW|MXC_F_RTC_FLAGS_PRESCALE_COMP); // clear all RTC TMR flags
    nextCompareCount = rtc_count | ((uint32_t)(0x1FFF));
    RTC_SetCompare(0, nextCompareCount);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);
    if((rtc_count & 0x1FFF)==0x1FFF){
        max_printf("---------------------------------------------wake-up was right before the next HAL.tick\r\n");
        //Wake-up was right before the next HAL.tick, but RTC0_IRQn was disabled. So set it here.
        NVIC_SetPendingIRQ(RTC0_IRQn);
    }
    //debug_printf("rtc_count:%x, rtc_rollover_counter:%x, rtc_hticks:%x, hticks:%x\r\n", rtc_count, rtc_rollover_counter, rtc_hticks, hticks);
#if CFG_watchdog
    if(xdelta > WD_OVERHEAD_FOR_SLEEP) {
        WDT2_EnableReset(WDT2_PERIOD_2_15_CLKS, MXC_V_WDT2_UNLOCK_KEY);  // 2^15/8KHz = 4.096s
        WDT2_Reset();
    }
#endif
    return;
}

// stubs from BasicsMac for STM32L to handle clocks in sleep mode
// used by both wake-up and init
/*static void clock_run (void) {
}

static void clock_sleep (int stype) {
}*/

static void clock_init() {
    CLKMAN_SetSystemClock(CLKMAN_SYSTEM_SOURCE_96MHZ, CLKMAN_SYSTEM_SCALE_DIV_1);
}

// NOTE: only call if interrupts are disabled
// 1 os_tick: 30.3us
// 32765 ticks per second
// RTC updates every 8*30.3us = 24.4us
#define hal_ticks_unsafe() ((u4_t) hal_xticks_unsafe())
static u8_t hal_xticks_unsafe (void) {
    u8_t xt = HAL.ticks;
    uint32_t count = RTC_GetCount(); //4096 ticks per second    count &= 0x1FFF;
    u4_t cnt = 8*count; //4096 * 8 = 32768 = 0xFFFF
    if ((RTC_GetFlags() & MXC_F_RTC_FLAGS_COMP0) != 0) {
        // max_printf("hal_xticks_unsafe overflow\r\n");
        // include pending overflow in evaluation
        count = RTC_GetCount();
        count &= 0x1FFF;
        cnt = 8*count;
        xt++;
    }
        return (xt << 16) | cnt;
}

u8_t hal_xticks () {
    hal_disableIRQs();
    u8_t xt = hal_xticks_unsafe();
    hal_enableIRQs();
    return xt;
}

u4_t hal_ticks () {
    return hal_xticks();
}

// NOTE: interrupts are already be disabled when this HAL function is called!
u1_t hal_sleep (u1_t type, u4_t targettime) {
    static const u8_t S_TH[] = {                        //overhead in os_ticks for sleep modes
          0, SLEEP_WFI_OVERHEAD, SLEEP_LP1_OVERHEAD     //WITH UART debug output.     4 (=0.48ms), 1500 (=46ms)
    };
    //Waking up time
    static const uint32_t WakingUpTime[] = {
            1, 4, 8
    };

    u8_t xnow = hal_xticks_unsafe();
    s4_t dt;
    if( type == HAL_SLEEP_FOREVER ) {
        dt = sec2osticks(12*60*60); // 12 h
    } else {
        dt = (s4_t) targettime - (s4_t) xnow;
    }
    if( dt <= 0 ) {
        return 0; // it's time now
    }


    // select sleep type
    int stype;
    for( stype = 0; stype < (HAL_SLEEP_CNT-1); stype++ ) {
        if( dt < S_TH[stype + 1] || HAL.maxsleep[stype] ) {
            break;
        }
    }

    // only use S2 if htt would be strictly larger
    if( stype == HAL_SLEEP_S2 ) {
        u8_t xtt = xnow + dt - S_TH[HAL_SLEEP_S2];
        if( (xtt >> 16) <= (xnow >> 16) ) {
            stype -= 1;
        }
    }

#ifdef CFG_rtstats
    static ostime_t wakeup;
    ostime_t t1 = xnow;
    ASSERT((t1 - wakeup) >= 0);
    HAL.rtstats.run += (t1 - wakeup);
#endif

    xnow += (dt - WakingUpTime[stype]);
    sleep(stype, xnow >> 16, xnow & 0xffff);

#ifdef CFG_rtstats
    ostime_t t2 = hal_ticks_unsafe();
    ASSERT((t2 - t1) >= 0);
    HAL.rtstats.sleep[stype] += (t2 - t1);
    wakeup = t2;
#endif

    return 1; // we slept
}

// short-term busy wait
// shouldn't be used for extended periods of time
// cannot possibly wait for more than 2 sec when interrupts are disabled because of timer overrun
void hal_waitUntil (u4_t time) {
    // assure waiting period is in intended range of up to 1 sec (and hasn't expired too long ago)
    ostime_t diff = time - hal_ticks();
    ASSERT(diff > -sec2osticks(1) && diff < sec2osticks(1));
    // busy wait until timestamp is reached
    while( ((s4_t) time - (s4_t) hal_ticks()) > 0 );
}

void hal_setMaxSleep (unsigned int level) {
    hal_disableIRQs();
    ASSERT(level < HAL_SLEEP_CNT-1);
    HAL.maxsleep[level] += 1;
    hal_enableIRQs();
}

void hal_clearMaxSleep (unsigned int level) {
    hal_disableIRQs();
    ASSERT(level < HAL_SLEEP_CNT-1);
    ASSERT(HAL.maxsleep[level]);
    HAL.maxsleep[level] -= 1;
    hal_enableIRQs();
}


// -----------------------------------------------------------------------------
// GPIO handling imported from Semtech's stm/gpio.c module.
// Rationale: gpio.h/c already used from Maxim

// empty for Maxim
static inline void gpio_begin (int port) {
}

// empty for Maxim
static inline void gpio_end (int port) {
}

void gpio_cfg_pin (int port, int pin, int gpiocfg) {
    max_printf("GPIO config for port: %x, pin: %x, cfgopt: %x, (", port, pin, gpiocfg);
    uint32_t gpio_func = GPIO_FUNC_GPIO;
    uint32_t pad = GPIO_PAD_NORMAL;
    if (((gpiocfg) & GPIOCFG_MODE_MASK)  == (GPIOCFG_MODE_INP)) {
        max_printf("INP, ");
        if(((gpiocfg) & GPIOCFG_PUPD_MASK)  == (GPIOCFG_PUPD_PUP)) {
            max_printf("PUP");
            pad = GPIO_PAD_INPUT_PULLUP;
        }
        else if (((gpiocfg) & GPIOCFG_PUPD_MASK)  == (GPIOCFG_PUPD_PDN)) {
            max_printf("PDN");
            pad = GPIO_PAD_INPUT_PULLDOWN;
        }
        else {
            max_printf("no PUP/PDN");
            pad = GPIO_PAD_INPUT;
        }
    }
    else if (((gpiocfg) & GPIOCFG_MODE_MASK)  == (GPIOCFG_MODE_OUT)) {
        max_printf("OUT, ");
            if(((gpiocfg) & GPIOCFG_OTYPE_MASK)  == (GPIOCFG_OTYPE_PUPD)) {
                max_printf("PUPD, ");
                if(((gpiocfg) & GPIOCFG_PUPD_MASK)  == (GPIOCFG_PUPD_PUP)) {
                    max_printf("PUP");
                    pad = GPIO_PAD_OPEN_DRAIN_PULLUP;
                }
                else if (((gpiocfg) & GPIOCFG_PUPD_MASK)  == (GPIOCFG_PUPD_PDN)) {
                    max_printf("PDN");
                    pad = GPIO_PAD_OPEN_SOURCE_PULLDOWN;
                }
                else {
                    max_printf("no PUP/PDN, ");
                    if(((gpiocfg) & GPIOCFG_OSPEED_MASK)  == (GPIOCFG_OSPEED_400kHz)) {
                        max_printf("400kHz");
                        pad = GPIO_PAD_SLOW;
                    }
                    else if(((gpiocfg) & GPIOCFG_OSPEED_MASK)  == (GPIOCFG_OSPEED_40MHz)) {
                        max_printf("40MHz");
                        pad = GPIO_PAD_FAST;
                    }
                    else {
                        max_printf("2MHz or 10MHz");
                        pad = GPIO_PAD_NORMAL;
                    }
                }
            }
        if(((gpiocfg) & GPIOCFG_OTYPE_MASK)  == (GPIOCFG_OTYPE_OPEN)) {
            if(((gpiocfg) & GPIOCFG_OSPEED_MASK)  == (GPIOCFG_OSPEED_400kHz)) {
                max_printf("400kHz");
                pad = GPIO_PAD_SLOW;
            }
            else if(((gpiocfg) & GPIOCFG_OSPEED_MASK)  == (GPIOCFG_OSPEED_40MHz)) {
                max_printf("40MHz");
                pad = GPIO_PAD_FAST;
            }
            else {
                max_printf("2MHz or 10MHz");
                pad = GPIO_PAD_NORMAL;
            }
        }
    }
    else if (((gpiocfg) & GPIOCFG_MODE_MASK)  == (GPIOCFG_MODE_ALT)) {
        max_printf("GPIO alternate function for port: %x, pin: %x, opt: %x", port, pin, gpiocfg & GPIOCFG_MODE_MASK);
        while(1);
    }
    else {
        pad = MXC_V_GPIO_OUT_MODE_HIGH_Z_INPUT_DISABLED;
    }

    max_printf(") translated to: %x\r\n", pad);
    const gpio_cfg_t gpio = {port, 1<<pin, gpio_func, pad};
    uint32_t err = GPIO_Config(&gpio);
    ASSERT(err==0);
#if 0
    gpio_begin(port);
    HW_CFG_PIN(GPIOx(port), pin, gpiocfg);
    gpio_end(port);
#endif
}

void gpio_set_pin (int port, int pin, int state) {
    gpio_begin(port);
    HW_SET_PIN(GPIOx(port), pin, state);
    gpio_end(port);
}

void gpio_cfg_set_pin (int port, int pin, int gpiocfg, int state) {
    gpio_begin(port);
    HW_SET_PIN(GPIOx(port), pin, state);
    HW_CFG_PIN(GPIOx(port), pin, gpiocfg);
    gpio_end(port);
}

int gpio_get_pin (int port, int pin) {
    int val;
    gpio_begin(port);
    val = HW_GET_PIN(GPIOx(port), pin);
    gpio_end(port);
    return val;
}

int gpio_transition (int port, int pin, int type, int duration, unsigned int config) {
    int val;
    gpio_begin(port);
    val = HW_GET_PIN(GPIOx(port), pin);
    HW_SET_PIN(GPIOx(port), pin, type);
    HW_CFG_PIN(GPIOx(port), pin, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_400kHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE);
    for (int i = 0; i < duration; i++) __NOP();
    HW_SET_PIN(GPIOx(port), pin, type ^ 1);
    for (int i = 0; i < duration; i++) __NOP();
    HW_CFG_PIN(GPIOx(port), pin, config);
    gpio_end(port);
    return val;
}

// Configuration of Interrupt
// Assumption: pin is already configured for input with or without pull-up or pull-down
void gpio_cfg_extirq_ex (int port, int pin, bool rising, bool falling) {
    max_printf("gpio_cfg_extirq_ex port: %x, pin:%x\r\n", port, pin);
    gpio_int_mode_t int_mode;
    const gpio_cfg_t gpio = {port, 1<<pin, 0, 0};
    if(rising) {
        if(falling) {
            int_mode = GPIO_INT_ANY_EDGE;
        }
        else {
            int_mode =  GPIO_INT_RISING_EDGE;
        }
    } else {
        if (falling){
            int_mode = GPIO_INT_FALLING_EDGE;
        }
        else {
            int_mode = GPIO_INT_DISABLE;
        }
    }

    GPIO_IntConfig(&gpio, int_mode);
    //GPIO_IntEnable(&gpio);
    GPIO_IntClr(&gpio);
    ASSERT(port < NB_OF_PORTS);
    NVIC_ClearPendingIRQ(GPIO_P0_IRQn + port);
    NVIC_EnableIRQ(GPIO_P0_IRQn + port);
    //Todo: configure interrupt priority
}

void gpio_cfg_extirq (int port, int pin, int irqcfg) {
    gpio_cfg_extirq_ex(port, pin,
            (irqcfg == GPIO_IRQ_CHANGE || irqcfg == GPIO_IRQ_RISING),
            (irqcfg == GPIO_IRQ_CHANGE || irqcfg == GPIO_IRQ_FALLING));
}


void gpio_set_extirq (int pin, int on) {
    const gpio_cfg_t gpio = {BRD_PORT(pin), 1<<(BRD_PIN(pin)), 0, 0};
    if (on) {
        GPIO_IntClr(&gpio);
        GPIO_IntEnable(&gpio);
    } else {
        GPIO_IntDisable(&gpio);
        GPIO_IntClr(&gpio);
    }
}

void pio_set (unsigned int pin, int value) {
    if (value < 0) {
        gpio_cfg_pin(BRD_PORT(pin), BRD_PIN(pin), 0
                | (((value & 1) == 0 ? GPIOCFG_PUPD_PUP : 0))
                | (((value & 2) == 0 ? GPIOCFG_PUPD_PDN : 0))
                | (((value & 4) == 0 ? GPIOCFG_MODE_ANA : GPIOCFG_MODE_INP)));
    } else {
        gpio_cfg_set_pin(BRD_PORT(pin), BRD_PIN(pin),
                GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE,
                value);
    }
}

int pio_get (unsigned int pin) {
    return gpio_get_pin(BRD_PORT(pin), BRD_PIN(pin));
}

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
#ifdef GPIO_DIO1
    CFG_PIN(GPIO_DIO1, GPIOCFG_MODE_INP | GPIOCFG_PUPD_NONE);
    IRQ_PIN(GPIO_DIO1, GPIO_IRQ_RISING);
#endif

#ifdef GPIO_BUSY
    CFG_PIN(GPIO_BUSY, GPIOCFG_MODE_INP | GPIOCFG_PUPD_NONE);
#endif

#ifdef GPIO_RST
    SET_PIN(GPIO_RST, 1);
    CFG_PIN(GPIO_RST, GPIOCFG_MODE_OUT | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
#endif

#ifdef GPIO_TXRX_EN //rf switch vdd
    SET_PIN_ONOFF(GPIO_TXRX_EN, 0);
    CFG_PIN(GPIO_TXRX_EN, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
#endif

#ifdef GPIO_DBG_LED
    //CFG_PIN(GPIO_DBG_LED, GPIOCFG_MODE_OUT | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PUP);
    //SET_PIN(GPIO_DBG_LED, 1);
    max_printf("debug_led enabled\r\n");
#endif
    //configure all GPIO as VDDIOH, but the RF GPIO
    //MXC_IOMAN->use_vddioh_0 = 0xFFFFFF07;
    MXC_IOMAN->use_vddioh_0 = 0xFFFFFF03;
    MXC_IOMAN->use_vddioh_1 = 0x1F;
    debug_printf("MXC_IOMAN->use_vddioh_1/0: %x, %x\r\n", MXC_IOMAN->use_vddioh_1, MXC_IOMAN->use_vddioh_0);
}

bool hal_pin_tcxo (u1_t val) {
#if defined(GPIO_TCXO_PWR)
    //not implemented yet. For the SX1261 this is done in radio-sx126x.c
    return true;
#else
    return false;
#endif
}

void hal_ant_switch (u1_t val) {
#ifdef SVC_pwrman
    static ostime_t t1;
    static int ctype;
    static uint32_t radio_ua;
    ostime_t now = hal_ticks();
    if( radio_ua ) {
        pwrman_consume(ctype, now - t1, radio_ua);
        radio_ua = 0;
    }
#endif
    if (val == HAL_ANTSW_OFF) {
#ifdef GPIO_TXRX_EN
    SET_PIN_ONOFF(GPIO_TXRX_EN, 0);
#endif
    } else {
#ifdef SVC_pwrman
        t1 = now;
        ctype = (val == HAL_ANTSW_RX) ? PWRMAN_C_RX : PWRMAN_C_TX;
        radio_ua = LMIC.radioPwr_ua;
#endif

#ifdef GPIO_TXRX_EN
        SET_PIN_ONOFF(GPIO_TXRX_EN, 1);
#endif
    }
#ifdef GPIO_RX
    //not implemented
#endif
#ifdef GPIO_TX
    //not implemented
#endif
#ifdef GPIO_TX2
    //not implemented
#endif
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
        SET_PIN(GPIO_RST, 0);
    } else { // keep floating
        SET_PIN(GPIO_RST, 1);  }
}

void hal_pin_busy_wait (void) {
    while (GET_PIN(GPIO_BUSY) != 0);
}

//copy pending GPIO interrupts to mask and clear them on HW.
#define DIO_UPDATE(dio,mask,time) do { \
    if( (MXC_GPIO->intfl[BRD_PORT(GPIO_DIO ## dio)] & (1 << BRD_PIN(GPIO_DIO ## dio))) ) { \
      MXC_GPIO->intfl[BRD_PORT(GPIO_DIO ## dio)] = (1 << BRD_PIN(GPIO_DIO ## dio)); \
        *(mask) |= (1 << dio); \
    } \
} while( 0 )

// generic EXTI IRQ handler for all channels
static void EXTI_IRQHandler () {
    u4_t now = hal_ticks_unsafe();
    u1_t diomask = 0;
    max_printf("EXTI at: %dms\r\n", osticks2ms(now));

#ifdef GPIO_DIO0
    // DIO 0
    DIO_UPDATE(0, &diomask, &now);
#endif

#ifdef GPIO_DIO1
    // DIO 1
    DIO_UPDATE(1, &diomask, &now);
#endif

#ifdef GPIO_DIO2
    // DIO 2
    DIO_UPDATE(2, &diomask, &now);
#endif

#ifdef GPIO_DIO3
    // DIO 3
    DIO_UPDATE(3, &diomask, &now);
#endif

    if(diomask) {
        // invoke radio handler (on IRQ)
        radio_irq_handler(diomask, now);
    }

#ifdef CFG_EXTI_IRQ_HANDLER
    // invoke user-defined interrupt handler
    {
        extern void CFG_EXTI_IRQ_HANDLER(void);
        CFG_EXTI_IRQ_HANDLER();
    }
#endif // CFG_EXTI_IRQ_HANDLER
    NVIC_ClearPendingIRQ(GPIO_P0_IRQn + 0);
    NVIC_ClearPendingIRQ(GPIO_P0_IRQn + 1);
    NVIC_ClearPendingIRQ(GPIO_P0_IRQn + 2);
    NVIC_ClearPendingIRQ(GPIO_P0_IRQn + 3);
    NVIC_ClearPendingIRQ(GPIO_P0_IRQn + 4);

}

static void dio_config (int mask, int pin, int gpio) {
    if( mask & pin ) {
#if 0
        if( BRD_GPIO_GET_CHAN(gpio) ) {
            unsigned int ch = BRD_GPIO_GET_CHAN(gpio) - 1;
            TIMx->CCMR1 |= (1 << (ch << 3));
            TIMx->SR = ~(TIM_SR_CC1IF << ch);
            TIMx->CCER |= (TIM_CCER_CC1E << (ch << 2));
            CFG_PIN_AF(gpio, 0);
        } else
#endif
        {
            //CFG_PIN(gpio, GPIOCFG_MODE_INP); //done in io_init
        }
        IRQ_PIN_SET(gpio, 1);
    } else {
#if 0
        if( BRD_GPIO_GET_CHAN(gpio) ) {
            unsigned int ch = BRD_GPIO_GET_CHAN(gpio) - 1;
            TIMx->CCER &= ~(TIM_CCER_CC1E << (ch << 2));
        }
#endif
        IRQ_PIN_SET(gpio, 0);
    }
}

void hal_irqmask_set (int mask) {
    static int prevmask = 0;

#ifdef GPIO_DIO0
    dio_config(mask, HAL_IRQMASK_DIO0, GPIO_DIO0);
#endif
#ifdef GPIO_DIO1
    dio_config(mask, HAL_IRQMASK_DIO1, GPIO_DIO1);
#endif
#ifdef GPIO_DIO2
    dio_config(mask, HAL_IRQMASK_DIO2, GPIO_DIO2);
#endif
#ifdef GPIO_DIO3
    dio_config(mask, HAL_IRQMASK_DIO3, GPIO_DIO3);
#endif

    if(mask==0){
  //GPIO_IntDisable(&dio1_pin);
    }

    mask = (mask != 0);
    if (prevmask != mask) {
        // prevent sleep if we are waiting for radio interrupts
        // TODO - evaluate if that is the correct thing to do
        if (mask) {
            hal_setMaxSleep(HAL_SLEEP_S0);
        } else {
            hal_clearMaxSleep(HAL_SLEEP_S0);
        }
        prevmask = mask;
    }
}


// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
    __disable_irq();
    HAL.irqlevel++;
}

void hal_enableIRQs () {
    if(--HAL.irqlevel == 0) {
        __enable_irq();
    }

}

#ifdef CFG_rtstats
void hal_rtstats_collect (hal_rtstats* stats) {
    stats->run_ticks = HAL.rtstats.run;
    HAL.rtstats.run = 0;
    for( int i = 0; i < HAL_SLEEP_CNT; i++ ) {
        stats->sleep_ticks[i] = HAL.rtstats.sleep[i];
        HAL.rtstats.sleep[i] = 0;
    }
}
#endif


#ifdef BRD_borlevel
// -----------------------------------------------------------------------------
// Brown-out reset

static void setbrownout (int level) {
#if 0
  unsigned int user = OB->USER & 0xffff;
    if( (user & 0xf) != level ) {
        write_optbyte(&OB->USER, (user & ~0xf) | level);
        // not reached
        ASSERT(0);
    }
#endif
}

#endif

// -----------------------------------------------------------------------------
// Random Number Generation
void hal_prng_init(void){
#ifdef ADC_AS_TRNG
  return;
#endif
  PRNG_Init();
  while(PRNG_Ready()==0);
}

void trng_next (uint32_t* dest, int count) {
#ifdef ADC_AS_TRNG
  *dest = (adc_read(0, 1) & 0xFF)<<24 | (adc_read(0, 1) & 0xFF)<<16 | (adc_read(0, 1) & 0xFF)<<8 | (adc_read(0, 1) & 0xFF);
  max_printf("trng_next: %x\r\n", *dest);
  return;
#endif
  while( count-- > 0 ) {
        *dest++ = PRNG_GetSeed()<<16 | PRNG_GetSeed();
    }
}

// -----------------------------------------------------------------------------
// Tests
#define SLEEP_ITERATIONS 1000
void sleep_test(void){
    uint32_t ret;
    uint32_t i;
    uint32_t now = hal_ticks();
    uint32_t sleeptime = 0;

    max_printf("sleep_test\r\n");

#if 0 // this is used to test RTC counter roll-over
    RTC_Stop();
    RTC_SetCount(0xFFFF0000);
    HAL.ticks = 0x7FFF7;
    RTC_Start();

    RTC0_handler_Compare0();
    max_printf("hal_ticks: %x\r\n", hal_ticks());
    max_printf("hal_ticks: %x\r\n", hal_ticks());
    max_printf("hal_ticks: %x\r\n", hal_ticks());
    max_printf("hal_ticks: %x\r\n", hal_ticks());
#endif

  uint32_t sleeptimes[10] = {1100, 1200, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000};
  for(i=0;i<(SLEEP_ITERATIONS)+10;i++){
      now = hal_ticks();
      if(i<SLEEP_ITERATIONS){
          sleeptime = i;
      } else {
          sleeptime = sleeptimes[i-SLEEP_ITERATIONS];
      }
      hal_disableIRQs();
      ret = hal_sleep(0, now + sleeptime);
      hal_enableIRQs();
      max_printf("hal_sleep until now (%x ticks) + %d ticks (=%dms) returned: %x\r\n", now, sleeptime, osticks2ms(sleeptime), ret);
      while(UART_PrepForSleep(MXC_UART_GET_UART(0)) != 0); //flush UART
  }
}

static void hal_configure_pwrman_pwrseq(){
  MXC_PWRSEQ->msk_flags &= ~(MXC_F_PWRSEQ_MSK_FLAGS_PWR_POWER_FAIL
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_BOOT_FAIL
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_FLASH_DISCHARGE
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_VDD12_RST_BAD
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_VDD18_RST_BAD
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_VRTC_RST_BAD
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_VDDB_RST_BAD
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_TVDD12_RST_BAD
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_POR18Z_FAIL_LATCH
      | MXC_F_PWRSEQ_MSK_FLAGS_PWR_TVDD12_BAD
      );

  debug_printf("MXC_PWRSEQ->flags: %x\r\n", MXC_PWRSEQ->flags);
  debug_printf("MXC_PWRSEQ->msk_flags: %x\r\n", MXC_PWRSEQ->msk_flags);
  MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_LOW_POWER_MODE;
  MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_PULLUPS_ENABLED;
  //MXC_PWRMAN->pwr_rst_ctrl &= ~MXC_F_PWRMAN_PWR_RST_CTRL_IO_ACTIVE; //not good
  MXC_PWRMAN->pwr_rst_ctrl &= ~MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;
  debug_printf("MXC_PWRMAN->pwr_rst_ctrl: %x\r\n", MXC_PWRMAN->pwr_rst_ctrl);
  debug_printf("MXC_PWRMAN->svm_events: %x\r\n", MXC_PWRMAN->svm_events);
}
static void hal_configure_flash_controller(){
  /* Improve flash access timing */
  // Commented out by Miromico in system_max32625.c after advised by Maxim Integrated.
  // This Optimization makes the latest versions of the Flash Controller HW busy and the flash programmer won't program the uC.
  // This Optimization can be made later in the application code.
  // In order to rescue wrongly programmed uC one can write zero to the below register in the flash programmer script.

  MXC_FLC->perform |= (MXC_F_FLC_PERFORM_EN_BACK2BACK_RDS |
                       MXC_F_FLC_PERFORM_EN_MERGE_GRAB_GNT |
                       MXC_F_FLC_PERFORM_AUTO_TACC |
                       MXC_F_FLC_PERFORM_AUTO_CLKDIV);

}


static uint32_t hal_get_and_print_reset_source() {
  uint32_t reset_source = MXC_PWRMAN->pwr_rst_ctrl;
  max_printf("Reset source: ");
  if (reset_source & MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP) max_printf("ARM Core Lockup\r\n");
  if (reset_source & MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_ARM) max_printf("Firmware cmd ARM\r\n");
  if (reset_source & MXC_F_PWRMAN_PWR_RST_CTRL_WATCHDOG_TIMEOUT) max_printf("Watchdog\r\n");
  if (reset_source & MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_SYSMAN) max_printf("Firmware cmd SYSMAN\r\n");
  if (reset_source & MXC_F_PWRMAN_PWR_RST_CTRL_SRSTN_ASSERTION) max_printf("SRST\r\n");
  if (reset_source & MXC_F_PWRMAN_PWR_RST_CTRL_POR) max_printf("POR\r\n");
  HAL.reset = reset_source;
  BACKTRACE();
  TRACE_VAL(reset_source);
}

static uint32_t hal_print_hw_info(){
  max_printf("//MAX32625 HW info from PWRMAN:\r\n");
  max_printf("pwr_rst_ctrl: %x\r\n", MXC_PWRMAN->pwr_rst_ctrl);
  hal_get_and_print_reset_source();
  max_printf("svm_events: %x\r\n", MXC_PWRMAN->svm_events);
  max_printf("die_type: %x\r\n", MXC_PWRMAN->die_type);
  max_printf("base_part_num: %x\r\n", MXC_PWRMAN->base_part_num);
  MXC_PWRMAN->mask_id1 = 0;
  MXC_PWRMAN->mask_id1 = MXC_F_PWRMAN_MASK_ID1_MASK_ID_ENABLE;
  uint32_t mask0 = MXC_PWRMAN->mask_id0;
  MXC_PWRMAN->mask_id1 = 0;
  MXC_PWRMAN->mask_id1 = MXC_F_PWRMAN_MASK_ID1_MASK_ID_ENABLE;
  uint32_t mask1 = MXC_PWRMAN->mask_id1;
  max_printf("MXC_PWRMAN->mask_id0: %x\r\n", mask0);
  max_printf("MXC_PWRMAN->mask_id1: %x\r\n", mask1);
  max_printf("peripheral_reset: %x\r\n", MXC_PWRMAN->peripheral_reset);
  MXC_PWRMAN->peripheral_reset = 0;
}


#ifdef CFG_DEBUG
static void debug_init (void); // fwd decl
#endif

bool hal_getResetFlag (u4_t flag) {
    return (HAL.reset & flag);
}

void hal_init (void* bootarg) {
    memset(&HAL, 0x00, sizeof(HAL));
    HAL.boottab = bootarg;
    HAL.battlevel = MCMD_DEVS_BATT_NOINFO;
    SystemCoreClock = RO_FREQ;
#ifdef CFG_DEBUG
    debug_init();
#endif
    hal_print_hw_info();

    hal_configure_pwrman_pwrseq();

    hal_configure_flash_controller();
//#if 0
    MXC_USB->dev_cn = MXC_F_USB_DEV_CN_URST;
    MXC_USB->dev_cn = 0;
    MXC_USB->cn = 0;
//#endif

#if 0
    HAL.reset = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    ASSERT(HAL.boottab->version >= 0x105); // require bootloader v261
#endif

#ifdef BRD_borlevel
    setbrownout(BRD_borlevel);
#endif

    hal_disableIRQs();

    clock_init();

    pd_init();

    hal_io_init();

    hal_spi_init();

    time_init();

    hal_enableIRQs();

    hal_prng_init();

#ifdef CFG_DEBUG
    //hal_debug_led (1);
#endif

//Full register dump
#if 0
    uint32_t *addr;
    addr = 0x40000000;
    uint32_t i=0;
    while(addr <= 0x40007000){
      max_printf("%x: ", addr);
      max_printf("%x, ", *addr);
      *addr++;
      i++;
      if (i % 8 == 0){
        max_printf("\r\n");
      }
    }
    hal_sleep()

    //uint32_t *addr;
    addr = 0x40008000;
    //uint32_t i=0;
    i = 0;
    while(addr <= 0x40020000){
      max_printf("%x: ", addr);
      max_printf("%x, ", *addr);
      *addr++;
      i++;
      if (i % 8 == 0){
        max_printf("\r\n");
      }
    }
#endif

//#if 0 //sleep test
    //sleep_test();
//#endif

#if 0
    leds_init();
#endif
#ifdef BRD_USART
    //usart_init();
#endif
#ifdef BRD_VIBE_TIM
    vibe_init();
#endif

#if defined(SVC_eefs)
    eefs_init((void*) APPDATA_BASE, APPDATA_SZ);
#endif
}

#if defined(BRD_I2C)
extern void i2c_irq (void);
#endif

const irqdef HAL_irqdefs[] = {
    { RTC0_IRQn, RTC0_handler_Compare0 }, //MAXIM RTC
    { GPIO_P0_IRQn, EXTI_IRQHandler },
    { GPIO_P1_IRQn, EXTI_IRQHandler },
    { GPIO_P2_IRQn, EXTI_IRQHandler },
    { GPIO_P3_IRQn, EXTI_IRQHandler }, //
    { GPIO_P4_IRQn, EXTI_IRQHandler }, //



#if defined(BRD_I2C)
#if BRD_I2C == 0
    { I2CM0_IRQn, i2c_irq },
#endif
#endif

#if 0
#if defined(BRD_USART)
#if BRD_USART == 1
    { UART0_IRQn, usart_irq },
#elif BRD_USART == BRD_LPUART(1)
    { UART1_IRQn, usart_irq },
#endif
#endif
#endif

#if defined(BRD_PWM_TIM)
#if BRD_PWM_TIM == 3
    { TMR3_0_IRQn, pwm_irq },
#endif
#endif

#if defined(BRD_PIR_TIM)
#if BRD_PIR_TIM == 3
    { TMR3_0_IRQn, pir_tim_irq },
#endif
#endif

#if defined(BRD_LED_TIM)
#if BRD_LED_TIM == 2
    { TMR2_0_IRQn, leds_pwm_irq },
#endif
#endif

#if defined(BRD_IR_TIM)
    { TMR2_0_IRQn, ir_tim_irq },
#endif

    { ~0, NULL } // end of list
};

unsigned int crc32 (void* ptr, int nwords) {
    return HAL.boottab->crc32(ptr, nwords);
}

u1_t hal_getBattLevel (void) {
    return HAL.battlevel;
}

void hal_setBattLevel (u1_t level) {
    HAL.battlevel = level;
}



#ifdef CFG_DEBUG

static void debug_init (void) {
    uint32_t err;
    const uart_cfg_t uart_cfg = {
        .parity = UART_PARITY_DISABLE,
        .size = UART_DATA_SIZE_8_BITS,
        .extra_stop = 0,
        .cts = 0,
        .rts = 0,
        .baud = DBG_UART_BR
    };

    const sys_cfg_uart_t uart_sys_cfg = {
        .clk_scale = CLKMAN_SCALE_AUTO,
        .io_cfg = IOMAN_UART(BRD_DBG_UART, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0)
    };

    err = UART_Init(DBG_USART, &uart_cfg, &uart_sys_cfg);
    ASSERT(err==0);
    debug_printf("\r\n============== DEBUG STARTED ==============\r\n");
}



void hal_debug_str (const char* str) {
    int length=0;
    while(str[length++] != 0){}

    UART_Write(DBG_USART, str, length - 1);
}

void hal_debug_led (int val) {
#ifdef GPIO_DBG_LED
    if (val!=0) {
        MXC_GPIO->out_val[BRD_PORT(GPIO_DBG_LED)] |= 1 << BRD_PIN(GPIO_DBG_LED);
        //GPIO_OutSet(&debug_led_pin);
    }
    else {
        MXC_GPIO->out_val[BRD_PORT(GPIO_DBG_LED)] &= ~(1 << BRD_PIN(GPIO_DBG_LED));
        //GPIO_OutClr(&debug_led_pin);
    }
#endif
}
#endif



void hal_fwinfo (hal_fwi* fwi) {
    debug_printf("hal_fwinfo\r\n");
    fwi->blversion = HAL.boottab->version;

    extern volatile hal_fwhdr fwhdr;
    fwi->version = fwhdr.version;
    fwi->crc = fwhdr.boot.crc;
    //fwi->flashsz = FLASH_SZ; //Why this makes the ctrl hang???
}

u4_t hal_unique (void) {
  debug_printf("hal_unique: not implemented yet\r\n");
  return 0x12345678;
#if 0
    uint32_t unique[3] = {
        *((uint32_t*) UNIQUE_ID0),
        *((uint32_t*) UNIQUE_ID1),
        *((uint32_t*) UNIQUE_ID2)
    };
    return crc32(&unique, 3);
#endif
}

void hal_reboot (void) {
    NVIC_SystemReset();
    // not reached
    hal_failed();
}

// persistent storage of stack data
typedef struct {
    uint32_t    dnonce[4];      // dev nonce history
    uint32_t    jnonce[4];      // join nonce history
} pdata;



// Note: The next nonce is stored, and the writes are spread over 4 fields. At
// 100k write cycles, this will allow 400k join requests. The DevNonce counter
// is only 16 bits, so it will roll-over much earlier, but the counter can be
// restarted at 0 if/when the Join EUI changes.

u4_t hal_dnonce_next (void) {
#ifdef BRD_DEV_NONCE_WITH_RANDOM_GENERATOR
    return os_getRndU1()<<24 | os_getRndU1()<<16 | os_getRndU1()<<8 | os_getRndU1();
#else

    pdata* p = (pdata*) STACKDATA_BASE;
    if (p->dnonce[0]==0xFFFFFFFF || p->dnonce[1]==0xFFFFFFFF || p->dnonce[2]==0xFFFFFFFF || p->dnonce[3]==0xFFFFFFFF){
        hal_dnonce_clear();
    }

    int x = 0;
    while( x < 3 && p->dnonce[x] < p->dnonce[x + 1] ) {
        x += 1;
    }
    u4_t dn = p->dnonce[x];
    eeprom_write(p->dnonce + ((x + 1) & 3), dn + 1);
    return dn;
#endif
}

void hal_dnonce_clear (void) {
    pdata* p = (pdata*) STACKDATA_BASE;
    for( int i = 0; i < 4; i++) {
        eeprom_write(p->dnonce + i, 0);
    }
}

void sha256 (uint32_t* hash, const uint8_t* msg, uint32_t len) {
    HAL.boottab->sha256(hash, msg, len);
}

bool hal_set_update (void* ptr) {
    return HAL.boottab->update(ptr, NULL) == BOOT_OK;
}

void flash_write (void* dst, const void* src, unsigned int nwords, bool erase) {
    hal_disableIRQs();
    HAL.boottab->wr_flash(dst, src, nwords, erase);
    hal_enableIRQs();
}

void hal_logEv (uint8_t evcat, uint8_t evid, uint32_t evparam) {
    // XXX:TBD
}
