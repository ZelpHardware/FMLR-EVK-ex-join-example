// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2019 Miromico AG
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.



#include "bootloader_impl.h"
//#include "update.h"
#include "bootloader_hw.h"
#include "sha2.h"

#include "clkman.h"
#include "flc.h"
#include "crc.h"
#include "debug.h"
#include "gpio_regs.h"

extern const boot_boottab boottab;
typedef void (*starttype) (boot_boottab* boottab);

uint32_t eeprom_buffer[DATA_EEPROM_SIZE/4]; // used during flash page erase

// ------------------------------------------------
// CRC-32

__attribute__((section(".boot"), always_inline))
static inline uint32_t __rbit (uint32_t v) {
    __asm__("rbit %0, %0" : "=r"(v) : "r"(v));
    return v;
}

static uint32_t boot_crc32 (void* buf, uint32_t nwords) {

	uint32_t* src = buf;
	CRC32_Reseed(0xFFFFFFFF);
    for(uint32_t i = 0; i < nwords; i++) {
        CRC32_AddData(src[i]);
    }
	return CRC32_GetCRC();

}


// ------------------------------------------------
// GPIO and LED config and macros

#define PORT_0      (0)             /**< Port 0  Define*/
#define PORT_1      (1)             /**< Port 1  Define*/
#define PORT_2      (2)             /**< Port 2  Define*/
#define PORT_3      (3)             /**< Port 3  Define*/
#define PORT_4      (4)             /**< Port 4  Define*/
#define PORT_5      (5)             /**< Port 5  Define*/
#define PORT_6      (6)             /**< Port 6  Define*/
#define PORT_7      (7)             /**< Port 7  Define*/
#define PORT_8      (8)             /**< Port 8  Define*/
#define PORT_9      (9)             /**< Port 9  Define*/
#define PORT_10     (10)            /**< Port 10  Define*/
#define PORT_11     (11)            /**< Port 11  Define*/
#define PORT_12     (12)            /**< Port 12  Define*/
#define PORT_13     (13)            /**< Port 13  Define*/
#define PORT_14     (14)            /**< Port 14  Define*/
#define PORT_15     (15)            /**< Port 15  Define*/

#define PIN_0       (1 << 0)        /**< Pin 0 Define */
#define PIN_1       (1 << 1)        /**< Pin 1 Define */
#define PIN_2       (1 << 2)        /**< Pin 2 Define */
#define PIN_3       (1 << 3)        /**< Pin 3 Define */
#define PIN_4       (1 << 4)        /**< Pin 4 Define */
#define PIN_5       (1 << 5)        /**< Pin 5 Define */
#define PIN_6       (1 << 6)        /**< Pin 6 Define */
#define PIN_7       (1 << 7)        /**< Pin 7 Define */

typedef enum {
    GPIO_FUNC_GPIO  = MXC_V_GPIO_FUNC_SEL_MODE_GPIO,    /**< GPIO Function Selection */
    GPIO_FUNC_PT    = MXC_V_GPIO_FUNC_SEL_MODE_PT,      /**< Pulse Train Function Selection */
    GPIO_FUNC_TMR   = MXC_V_GPIO_FUNC_SEL_MODE_TMR      /**< Timer Function Selection */
}
gpio_func_t;

typedef enum {
    GPIO_PAD_INPUT_PULLUP           = MXC_V_GPIO_OUT_MODE_HIGH_Z_WEAK_PULLUP,       /**< Set pad to high impedance, weak pull-up */
    GPIO_PAD_OPEN_DRAIN             = MXC_V_GPIO_OUT_MODE_OPEN_DRAIN,               /**< Set pad to open-drain with high impedance with input buffer */
    GPIO_PAD_OPEN_DRAIN_PULLUP      = MXC_V_GPIO_OUT_MODE_OPEN_DRAIN_WEAK_PULLUP,   /**< Set pad to open-drain with weak pull-up */
    GPIO_PAD_INPUT                  = MXC_V_GPIO_OUT_MODE_NORMAL_HIGH_Z,            /**< Set pad to high impednace, input buffer enabled */
    GPIO_PAD_NORMAL                 = MXC_V_GPIO_OUT_MODE_NORMAL,                   /**< Set pad to normal drive mode for high an low output */
    GPIO_PAD_SLOW                   = MXC_V_GPIO_OUT_MODE_SLOW_DRIVE,               /**< Set pad to slow drive mode, which is normal mode with negative feedback to slow edge transitions */
    GPIO_PAD_FAST                   = MXC_V_GPIO_OUT_MODE_FAST_DRIVE,               /**< Set pad to fash drive mode, which is normal mode with a transistor drive to drive fast high and low */
    GPIO_PAD_INPUT_PULLDOWN         = MXC_V_GPIO_OUT_MODE_HIGH_Z_WEAK_PULLDOWN,     /**< Set pad to weak pulldown mode */
    GPIO_PAD_OPEN_SOURCE            = MXC_V_GPIO_OUT_MODE_OPEN_SOURCE,              /**< Set pad to open source mode, transistor drive to high */
    GPIO_PAD_OPEN_SOURCE_PULLDOWN   = MXC_V_GPIO_OUT_MODE_OPEN_SOURCE_WEAK_PULLDOWN /**< Set pad to open source with weak pulldown mode, transistor drive to high, weak pulldown to GND for low */
} gpio_pad_t;

typedef struct {
    uint32_t port;      /// Index of GPIO port
    uint32_t mask;      /// Pin mask. Multiple bits can be set.
    gpio_func_t func;   /// Function type
    gpio_pad_t pad;     /// Pad type
} gpio_cfg_t;

/******************************************************************************/
int SYS_GPIO_Init(void)
{
    if (CLKMAN_GetClkScale(CLKMAN_CLK_GPIO) == CLKMAN_SCALE_DISABLED) {
        CLKMAN_SetClkScale(CLKMAN_CLK_GPIO, CLKMAN_SCALE_DIV_1);
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
static int PinConfig(unsigned int port, unsigned int pin, gpio_func_t func, gpio_pad_t pad)
{
    /* Check if available */
    if (!(MXC_GPIO->free[port] & (1 << pin))) {
        return E_BUSY;
    }

    /* Set function */
    uint32_t func_sel = MXC_GPIO->func_sel[port];
    func_sel &= ~(0xF << (4 * pin));
    func_sel |=  (func << (4 * pin));
    MXC_GPIO->func_sel[port] = func_sel;

    /* Normal input is always enabled */
    MXC_GPIO->in_mode[port] &= ~(0xF << (4 * pin));

    /* Set requested output mode */
    uint32_t out_mode = MXC_GPIO->out_mode[port];
    out_mode &= ~(0xF << (4 * pin));
    out_mode |=  (pad << (4 * pin));
    MXC_GPIO->out_mode[port] = out_mode;

    /* Enable the pull up/down if necessary */
    if (pad == MXC_V_GPIO_OUT_MODE_HIGH_Z_WEAK_PULLUP) {
        MXC_GPIO->out_val[port] |= (1 << pin);
    } else if (pad == MXC_V_GPIO_OUT_MODE_HIGH_Z_WEAK_PULLDOWN) {
        MXC_GPIO->out_val[port] &= ~(1 << pin);
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int GPIO_Config(const gpio_cfg_t *cfg)
{
    unsigned int pin;
    int err = E_NO_ERROR;

    MXC_ASSERT(cfg);
    MXC_ASSERT(cfg->port < MXC_GPIO_NUM_PORTS);

    // Set system level configurations
    if ((err = SYS_GPIO_Init()) != E_NO_ERROR) {
        return err;
    }

    // Configure each pin in the mask
    for (pin = 0; pin < MXC_GPIO_MAX_PINS_PER_PORT; pin++) {
        if (cfg->mask & (1 << pin)) {
            if (PinConfig(cfg->port, pin, cfg->func, cfg->pad) != E_NO_ERROR) {
                err = E_BUSY;
            }
        }
    }

    return err;
}

const gpio_cfg_t led_pin0 = { PORT_3, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_OPEN_DRAIN };
const gpio_cfg_t led_pin1 = { PORT_3, PIN_1, GPIO_FUNC_GPIO, GPIO_PAD_OPEN_DRAIN };
const gpio_cfg_t led_pin2 = { PORT_3, PIN_2, GPIO_FUNC_GPIO, GPIO_PAD_OPEN_DRAIN };
const gpio_cfg_t led_pin3 = { PORT_3, PIN_3, GPIO_FUNC_GPIO, GPIO_PAD_OPEN_DRAIN };

void bootloader_GPIO_OutSet(const gpio_cfg_t *cfg)
{
    MXC_GPIO->out_val[cfg->port] |= cfg->mask;
}

void bootloader_GPIO_OutClr(const gpio_cfg_t *cfg)
{
    MXC_GPIO->out_val[cfg->port] &= ~(cfg->mask);
}


#define LED_INIT(gpio)   	GPIO_Config(gpio)
#define LED_ON(gpio)        bootloader_GPIO_OutClr(gpio)
#define LED_OFF(gpio)       bootloader_GPIO_OutSet(gpio)

// ------------------------------------------------
// Panic handler

#if defined(BOOT_LED_GPIO)

extern void delay (int); // provided by util.S

// delay and refresh watchdog
static void pause (int v) {
    // refresh watchdog
    IWDG->KR = 0xaaaa;
    // pause
    delay(v);
}

static void blink_value (uint32_t v) {
    // blink nibble-by-nibble
    // least-significant-nibble first, 0x0 -> 1 blink, 0xf -> 16 blinks
    do {
	uint32_t n = v & 0xf;
	do {
	    LED_ON(BOOT_LED_GPIO);
	    pause(6);
	    LED_OFF(BOOT_LED_GPIO);
	    pause(6);
	} while (n--);
	v >>= 4;
	pause(12);
    } while (v);
}
#endif

// force inlining of reset call
__attribute__((always_inline)) static void NVIC_SystemReset (void);

__attribute__((noreturn))
void boot_panic (uint32_t type, uint32_t reason, uint32_t addr) {
	debug_printf("panic handler\r\n");
#if 0
	// disable all interrupts
    __disable_irq();
    // startup MSI @2.1MHz
    RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | RCC_ICSCR_MSIRANGE_5;
    RCC->CR |= RCC_CR_MSION;
    while ((RCC->CR & RCC_CR_MSIRDY) == 0);
    // switch clock source to MSI
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);
    // no flash wait states
    FLASH->ACR &= ~FLASH_ACR_LATENCY;

#if defined(BOOT_LED_GPIO)
    LED_INIT(BOOT_LED_GPIO);

    int repeat = 3;
    while (repeat-- > 0) {
	// blink long
	LED_ON(BOOT_LED_GPIO);
	pause(30);
	LED_OFF(BOOT_LED_GPIO);
	pause(30);
	// blink type
	blink_value(type);
	pause(30);
	// blink reason
	blink_value(reason);
	pause(30);
	// blink address
	blink_value(addr);
	pause(30);
    }
#endif
    NVIC_SystemReset();
    // not reached
#endif
    while (1);
}

__attribute__((noreturn, naked))
static void fw_panic (uint32_t reason, uint32_t addr) {
    boot_panic(BOOT_PANIC_TYPE_FIRMWARE, reason, addr);
}

// ------------------------------------------------
// Flash functions

typedef void (*wr_fl_hp) (uint32_t*, const uint32_t*);

typedef struct {
    boot_uphdr* fwup;
    wr_fl_hp wf_func;
} up_ctx;

static void unlock_flash (void) {
}

static void relock_flash (void) {
}



extern uint32_t wr_fl_hp_begin;	// provided by util.S
extern uint32_t wr_fl_hp_end;	// provided by util.S

#define WR_FL_HP_WORDS	(&wr_fl_hp_end - &wr_fl_hp_begin)

static wr_fl_hp prep_wr_fl_hp (uint32_t* funcbuf) {
    for (int i = 0; i < WR_FL_HP_WORDS; i++) {
	funcbuf[i] = (&wr_fl_hp_begin)[i];
    }
    return THUMB_FUNC(funcbuf);
}

static void fl_write (wr_fl_hp wf_func, uint32_t* dst, const uint32_t* src, uint32_t nwords, bool erase) {
	int err;

	err = FLC_Write((uint32_t)dst, src, (nwords*4), (uint8_t)0x2);

	if (err!= E_NO_ERROR){
	debug_printf("Could not write to flash\r\n");
	boot_panic(BOOT_PANIC_TYPE_BOOTLOADER, BOOT_PANIC_REASON_FLASH, 0);
	}

}
static void write_flash (uint32_t* dst, const uint32_t* src, uint32_t nwords, bool erase) {

	int err;

	err = FLC_Write((uint32_t)dst, src, (nwords*4), (uint8_t)0x2);

	if (err!= E_NO_ERROR){
	debug_printf("Could not write to flash\r\n");
	boot_panic(BOOT_PANIC_TYPE_BOOTLOADER, BOOT_PANIC_REASON_FLASH, 0);
	}
}

static void ee_write (uint32_t* dst, uint32_t val) {

	uint32_t *eeprom_pointer = (uint32_t*)(DATA_EEPROM_BASE);
	uint32_t *ram_pointer = eeprom_buffer;

	for (int i=0; i< (DATA_EEPROM_SIZE/4); i++){
		*ram_pointer = *eeprom_pointer;
		ram_pointer++;
		eeprom_pointer++;
	}

	*((uint32_t*)(dst - (uint32_t*)(DATA_EEPROM_BASE) + eeprom_buffer)) = val;
//original code
#if 0
	*dst = val;
    while (FLASH->SR & FLASH_SR_BSY);
#endif
    write_flash((uint32_t*)(DATA_EEPROM_BASE), eeprom_buffer, DATA_EEPROM_SIZE/4, true);
}


// ------------------------------------------------
// Update glue functions
uint32_t up_install_init (void* ctx, uint32_t size, void** pdst) {
    up_ctx* uc = ctx;
    if (ROUND_PAGE_SZ(size) > ((uintptr_t) uc->fwup - BOOT_FW_BASE)) {
  // new firmware would overwrite update
  return BOOT_E_SIZE;
    }
    *pdst = (void*) BOOT_FW_BASE;
    return BOOT_OK;
}

void up_flash_wr_page (void* ctx, void* dst, void* src) {
    up_ctx* uc = ctx;
#if defined(UPDATE_LED_GPIO)
    LED_ON(UPDATE_LED_GPIO);
#endif
    fl_write(uc->wf_func, dst, src, FLASH_PAGE_SZ >> 2, true);
#if defined(UPDATE_LED_GPIO)
    LED_OFF(UPDATE_LED_GPIO);
#endif
}

void up_flash_unlock (void* ctx) {
#if defined(UPDATE_LED_GPIO)
    LED_INIT(UPDATE_LED_GPIO);
#endif
    unlock_flash();
}

void up_flash_lock (void* ctx) {
    relock_flash();
#if defined(UPDATE_LED_GPIO)
    LED_DEINIT(UPDATE_LED_GPIO);
#endif
}


// ------------------------------------------------
// Update functions
static void do_install (boot_uphdr* fwup) {
    uint32_t funcbuf[WR_FL_HP_WORDS];
    up_ctx uc = {
	.wf_func = prep_wr_fl_hp(funcbuf),
	.fwup = fwup,
    };
    if (update(&uc, fwup, true) != BOOT_OK) {
	boot_panic(BOOT_PANIC_TYPE_BOOTLOADER, BOOT_PANIC_REASON_FLASH, 0);
    }
}
static bool check_update (boot_uphdr* fwup) {
	uint32_t flash_sz = FLASH_SZ();
    return ( ((intptr_t) fwup & 3) == 0
	     && (intptr_t) fwup >= FLASH_BASE
	     && sizeof(boot_uphdr) <= flash_sz - ((intptr_t) fwup - FLASH_BASE)
	     && fwup->size >= sizeof(boot_uphdr)
	     && (fwup->size & 3) == 0
	     && fwup->size <= flash_sz - ((intptr_t) fwup - FLASH_BASE)
	     && boot_crc32(((unsigned char*) fwup) + 8, (fwup->size - 8) >> 2) == fwup->crc
	     && true /* TODO hardware id match */ );
    return false;
    }

static uint32_t set_update (void* ptr, hash32* hash) {
	bootloader_GPIO_OutClr(&led_pin1);
	uint32_t rv;
    if (ptr == NULL) {
	rv = BOOT_OK;
    } else {
        up_ctx uc = {
            .fwup = ptr,
        };
	rv = check_update((boot_uphdr*) ptr) ? update(&uc, ptr, false) : BOOT_E_SIZE;
    }
    if (rv == BOOT_OK) {
	boot_config* cfg = (boot_config*) BOOT_CONFIG_BASE;
	// unlock EEPROM
//	FLASH->PEKEYR = 0x89ABCDEF; // FLASH_PEKEY1
//	FLASH->PEKEYR = 0x02030405; // FLASH_PEKEY2
	// copy hash
	if (hash) {
	    for (int i = 0; i < 8; i++) {
		ee_write(&cfg->hash.w[i], hash->w[i]);
	    }
	}
	// set update pointer
	ee_write(&cfg->fwupdate1, (uint32_t) ptr);
	ee_write(&cfg->fwupdate2, (uint32_t) ptr);
	// relock EEPROM
//	FLASH->PECR |= FLASH_PECR_PELOCK;
    }
    return rv;
}

// ------------------------------------------------
// Bootloader main entry point

void bootloader_burn_us(int val){
	for (volatile int i=0; i<val; i++)
	{
		for (volatile int j=0; j<7370; j++);
	}
}

extern uint32_t SystemCoreClock;

static void clock_init() {
	SystemCoreClock = RO_FREQ/2;
	CLKMAN_SetSystemClock(CLKMAN_SYSTEM_SOURCE_96MHZ, CLKMAN_SYSTEM_SCALE_DIV_1);
}

void* bootloader (void) {

	int err;
	clock_init();
#ifdef CFG_DEBUG
    debug_init();
    debug_printf("Bootloader\r\n");
#endif
    err = FLC_Init();
    if (err!=E_NO_ERROR){
	debug_printf("Could not initialize Flash Controller\r\n");
	boot_panic(BOOT_PANIC_TYPE_BOOTLOADER, BOOT_PANIC_REASON_FLASH, 0);
    }

    CRC32_Init(1);

    GPIO_Config(&led_pin0);
	GPIO_Config(&led_pin1);
	GPIO_Config(&led_pin2);
	GPIO_Config(&led_pin3);
    bootloader_GPIO_OutSet(&led_pin0);
    bootloader_GPIO_OutSet(&led_pin1);
    bootloader_GPIO_OutSet(&led_pin2);
    bootloader_GPIO_OutSet(&led_pin3);

    boot_fwhdr* fwh = (boot_fwhdr*) BOOT_FW_BASE;
    boot_config* cfg = (boot_config*) BOOT_CONFIG_BASE;

    debug_printf("cfg->fwupdate1: %x\r\n", cfg->fwupdate1);
    debug_printf("cfg->fwupdate2: %x\r\n", cfg->fwupdate2);
    debug_printf("going to check presence of fwupdate...\r\n");

    // check presence and integrity of firmware update
    if (cfg->fwupdate1 == cfg->fwupdate2) {
	boot_uphdr* fwup = (boot_uphdr*) cfg->fwupdate1;
	if (fwup != NULL && check_update(fwup)) {
	    do_install(fwup);
	}
    }

    debug_printf("no update, so going to check integrity of FW...\r\n");

    // verify integrity of current firmware
    if (fwh->size < sizeof(boot_fwhdr)
	    || fwh->size > (FLASH_SZ() - (BOOT_FW_BASE - FLASH_BASE))
	    || boot_crc32(((unsigned char*) fwh) + 8, (fwh->size - 8) >> 2) != fwh->crc) {
	boot_panic(BOOT_PANIC_TYPE_BOOTLOADER, BOOT_PANIC_REASON_CRC, 0);
    }

    debug_printf("FW integrity OK, going to clear fwupdate pointer...\r\n");


    // clear fwup pointer in EEPROM if set
    if (cfg->fwupdate1 != 0 || cfg->fwupdate2 != 0) {
	set_update(NULL, NULL);
    }
    debug_printf("value of entrypoint: %x\r\n", fwh->entrypoint);
	bootloader_burn_us(1000);
    starttype thestart;
    thestart = (starttype)(fwh->entrypoint);
    thestart(&boottab);
    debug_printf("should not be reached\r\n");
    return 0;
}

// ------------------------------------------------
// Bootloader information table
//
// Version history:
//
//   0x100 - initial version
//   0x101 - added wr_flash
//   0x102 - added sha_256
//   0x103 - support for self-contained LZ4 updates
//   0x104 - support for LZ4 block-delta updates
//   0x105 - wr_flash: allow flash erase-only operation by setting src=NULL

__attribute__((section(".boot.boottab"))) const boot_boottab boottab = {
    .version	= 0x108,
    .update	= set_update,
    .panic	= fw_panic,
    .crc32      = boot_crc32,
    .wr_flash   = write_flash,
    .sha256     = sha256,
};
