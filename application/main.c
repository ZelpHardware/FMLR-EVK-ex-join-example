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

#include "lmic.h"
#include "lwmux/lwmux.h"
#include "uexti/uexti.h"

#include "drivers.h"
#include "driver_types.h"

//button timeout used at start-up
#define BUTTON_TIMEOUT 100
#define DRIVER_sht3x
#define STM32L071xx

//---------------------------
//Button handling (asynchronous):
//---------------------------
static osjob_t button_job;
static lwm_job button_tx_job;
uint32_t buttonDebounceTime = 500;
uint8_t button_pressed_state = 0;
uint8_t button_press_sending_state = 0;

#ifdef DRIVER_sht21
static sht21_data sht;
#endif

#ifdef DRIVER_sht3x
static sht3x_data sht3x;
#endif


static void init_button(uint32_t b) {
    // button interrupt on rising edge (on press)
    CFG_PIN(b, GPIOCFG_MODE_INP | GPIOCFG_PUPD_NONE);
    uexti_config(b, false, true);
    uexti_enable(b, true); // enable
}

static void deinit_button(uint32_t b) {
    // button interrupt on rising edge (on press)
    CFG_PIN(b, GPIOCFG_MODE_ANA | GPIOCFG_PUPD_NONE);
    //IRQ_PIN(b, GPIO_IRQ_FALLING);
    IRQ_PIN_SET(b, 0);
    uexti_config(b, false, false);
    uexti_enable(b, false); // enable
}

static void txc_async_example (void) {
    debug_printf("TX completed (async example)\r\n");
    button_press_sending_state = 0;
}

// send status message
static bool tx_button_press_example(lwm_txinfo* txinfo) {
    BACKTRACE();
    u1_t idx = 0;
    txinfo->data[idx++] = 0xAA;
    txinfo->data[idx++] = 0xBB;
    txinfo->data[idx++] = 0xCC;
    txinfo->data[idx++] = 0xDD;
    txinfo->data[idx++] = 0xEE;
    txinfo->data[idx++] = 0xFF;

    txinfo->dlen = idx;
    txinfo->port = 15;
    txinfo->txcomplete = txc_async_example;
    txinfo->confirmed = 1;
    return true;
}

static void button_handler(osjob_t* job) {
    debug_printf("button_handler\r\n");
    button_pressed_state = 0;
    if (button_press_sending_state == 0) {
        debug_printf("sending LoRa message\r\n");
        lwm_request_send(&button_tx_job, 10, tx_button_press_example);
    }
    else {
        debug_printf("LoRa message sending pending... drop this event\r\n");
    }
}

#ifdef STM32L0
#pragma message (  "*************** STM32L0 is defined ******************" )
void button_irq(unsigned int* mask) {
    if (*mask & (1 << PIO_IRQ_LINE(GPIO_BUTTON_DEVBOARD))) {
        if (button_pressed_state == 0){
            *mask &= ~(1 << PIO_IRQ_LINE(GPIO_BUTTON_DEVBOARD));
            button_pressed_state = 1;
            debug_printf("%x, %x\r\n", BRD_PORT(GPIO_BUTTON_DEVBOARD), BRD_PIN(GPIO_BUTTON_DEVBOARD));
            os_setApproxTimedCallback(&button_job, os_getTime() + ms2osticks(buttonDebounceTime), button_handler);
        }
        else {
            debug_printf("debouncing\r\n");
        }
    }
}
#endif

#if MAX32625
void button_irq(unsigned int* mask) {
    debug_printf("b");

    unsigned int old_mask[NB_OF_PORTS];
    memcpy(&old_mask, mask, NB_OF_PORTS * sizeof(uint32_t));

    if (mask[BRD_PORT(GPIO_BUTTON_DEVBOARD)] & (1 << BRD_PIN(GPIO_BUTTON_DEVBOARD))) {
        // mask[BRD_PORT(GPIO_BUTTON_DEVBOARD)] &= ~(1 << BRD_PIN(GPIO_BUTTON_DEVBOARD));
        if (button_pressed_state == 0){
            mask[BRD_PORT(GPIO_BUTTON_DEVBOARD)] &= ~(1 << BRD_PIN(GPIO_BUTTON_DEVBOARD));
            button_pressed_state = 1;
            // disable_button(GPIO_BUTTON_DEVBOARD);
            deinit_button(GPIO_BUTTON_DEVBOARD);
            debug_printf("%x, %x\r\n", BRD_PORT(GPIO_BUTTON_DEVBOARD), BRD_PIN(GPIO_BUTTON_DEVBOARD));
            os_setApproxTimedCallback(&button_job, os_getTime() + ms2osticks(buttonDebounceTime), button_handler);
        }
        else {
            debug_printf("debouncing\r\n");
        }
    }
}
#endif

static lwm_job lj;
static osjob_t* mainjob;

static void next (osjob_t* job);

static void txc_periodic_example (void) {
    debug_printf("TX of periodic example completed\r\n");
    os_setApproxTimedCallback(mainjob, os_getTime() + sec2osticks(30), next);
}

static bool tx_periodic_example (lwm_txinfo* txinfo) {
    txinfo->data = (unsigned char*) "hello";
    txinfo->dlen = 5;
    txinfo->port = 15;
    txinfo->txcomplete = txc_periodic_example;
    return true;
}

static void next (osjob_t* job) {
    lwm_request_send(&lj, 0, tx_periodic_example);
}

static void device_init(osjob_t* job) {
    static enum {
        INIT, G, B, R, LED_OFF,
#ifdef DRIVER_mx25
        MX25_INIT_STEPS,
#endif
#ifdef DRIVER_sht21
        SHT21_INIT_STEPS,
        SHT21_START_READ_OUT_STEP,
        SHT21_END_READ_OUT_STEP,
#endif
#ifdef DRIVER_sht3x
        SHT3X_INIT_STEPS,
        SHT3X_START_READ_OUT_STEP,
        SHT3X_END_READ_OUT_STEP,
#endif
        COMPLETED
    } step;

    static eDeviceStatus_t status;

    switch (step) {
    case INIT:
      debug_printf("Configure RGB LED\r\n");
      CFG_PIN(GPIO_LEDR, GPIOCFG_MODE_OUT | GPIOCFG_PUPD_NONE);
      CFG_PIN(GPIO_LEDG, GPIOCFG_MODE_OUT | GPIOCFG_PUPD_NONE);
      CFG_PIN(GPIO_LEDB, GPIOCFG_MODE_OUT | GPIOCFG_PUPD_NONE);
      step++;

      // fall-through
    case R:
      debug_printf("R, ");
      SET_PIN(GPIO_LEDR, 0);
      SET_PIN(GPIO_LEDG, 1);
      SET_PIN(GPIO_LEDB, 1);
      os_setTimedCallback(job, os_getTime() + sec2osticks(1), device_init);
      break;

    case G:
      debug_printf("G, ");
      SET_PIN(GPIO_LEDR, 1);
      SET_PIN(GPIO_LEDG, 0);
      SET_PIN(GPIO_LEDB, 1);
      os_setTimedCallback(job, os_getTime() + sec2osticks(1), device_init);
      break;

    case B:
      debug_printf("B\r\n");
      SET_PIN(GPIO_LEDR, 1);
      SET_PIN(GPIO_LEDG, 1);
      SET_PIN(GPIO_LEDB, 0);
      os_setTimedCallback(job, os_getTime() + sec2osticks(1), device_init);
      break;

    case LED_OFF:
        SET_PIN(GPIO_LEDR, 1);
        SET_PIN(GPIO_LEDG, 1);
        SET_PIN(GPIO_LEDB, 1);
        // fall-through

#ifdef DRIVER_mx25
    case MX25_INIT:
        mx25_init(job, device_init, &status);
        break;

    case MX25_DONE:
        if (status != eDevice_Initialized) {
            debug_printf("mx25: init failed\r\n");
        } else {
            debug_printf("mx25: init done\r\n");
        }
        step++;
#endif

#ifdef DRIVER_sht21
    case SHT21_INIT:
        debug_printf("sht21: init\r\n");
        sht21_init(job, device_init, &status);
        break;

    case SHT21_DONE:
        if (status != eDevice_Initialized) {
            debug_printf("sht21: init failed\r\n");
        } else {
            debug_printf("sht21: init done\r\n");
        }
        step++;

    // fall-through
    case SHT21_START_READ_OUT_STEP:
      debug_printf("SHT21_START_READ_OUT_STEPS\r\n");
      sht21_read(job, device_init, &status, &sht);
      break;

    case SHT21_END_READ_OUT_STEP:
      debug_printf("sht21: t: %d, rh: %d\r\n", sht.temp, sht.rh);
      os_setCallback(job, device_init);
      break;
#endif

#ifdef DRIVER_sht3x
    case SHT3X_INIT:
        debug_printf("sht3x: init\r\n");
        sht3x_init(job, device_init, &status);
        break;

    case SHT3X_DONE:
        if (status != eDevice_Initialized) {
            debug_printf("sht3x: init failed\r\n");
        } else {
            debug_printf("sht3x: init done\r\n");
        }
        step++;

    // fall-through
    case SHT3X_START_READ_OUT_STEP:
      debug_printf("SHT3X_START_READ_OUT_STEPS\r\n");
      sht3x_read(job, device_init, &status, &sht3x);
      break;

    case SHT3X_END_READ_OUT_STEP:
      debug_printf("sht3x: t: %d, rh: %d\r\n", sht3x.temp, sht3x.rh);
      os_setCallback(job, device_init);
      break;
#endif


    case COMPLETED:
        //finally initialize the button for async events
        init_button(GPIO_BUTTON_DEVBOARD);
        step = INIT;
        return;
    }
    step += 1;
}

// service hook (invoked before downlink hook)
void app_event(ev_t e) {
    switch (e) {
        case EV_JOINED:
            debug_printf("joined\r\n");
            break;
        default:
            break;
        }
}

void app_dl (int port, unsigned char* data, int dlen, unsigned int flags) {
    debug_printf("DL[%d]: %h\r\n", port, data, dlen);
}

void app_main (osjob_t* job) {
    debug_printf("Hello World!\r\n");
    // Flash FMLR LED
    SET_PIN(GPIO_LED1, 0);
    os_setTimedCallback(job, os_getTime() + sec2osticks(1), device_init);
    SET_PIN(GPIO_LED1, 1);
    os_setTimedCallback(job, os_getTime() + sec2osticks(1), device_init);
 

    unsigned char eui[8];
    os_getDevEui(eui);
    debug_printf("id: %E \r\n",eui);

    // join network
    lwm_setmode(LWM_MODE_NORMAL);

    // re-use current job
    mainjob = job;

    // init application
    os_setCallback(mainjob, device_init);
}
