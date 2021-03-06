/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2016-03-04 15:15:48 -0600 (Fri, 04 Mar 2016) $
 * $Revision: 21711 $
 *
 ******************************************************************************/

#ifndef _MXC_PWRMAN_REGS_H_
#define _MXC_PWRMAN_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif


/**
 * @brief Defines PAD Modes for Wake Up Detection.
 */
typedef enum {
    /** WUD Mode for Selected PAD = Clear/Activate */
    MXC_E_PWRMAN_PAD_MODE_CLEAR_SET,
    /** WUD Mode for Selected PAD = Set WUD Act Hi/Set WUD Act Lo */
    MXC_E_PWRMAN_PAD_MODE_ACT_HI_LO,
    /** WUD Mode for Selected PAD = Set Weak Hi/ Set Weak Lo */
    MXC_E_PWRMAN_PAD_MODE_WEAK_HI_LO,
    /** WUD Mode for Selected PAD = No pad state change */
    MXC_E_PWRMAN_PAD_MODE_NONE
} mxc_pwrman_pad_mode_t;

/*
   Typedefed structure(s) for module registers (per instance or section) with direct 32-bit
   access to each register in module.
*/

/*                                                          Offset          Register Description
                                                            =============   ============================================================================ */
typedef struct {
    __IO uint32_t pwr_rst_ctrl;                         /*  0x0000          Power Reset Control and Status                                               */
    __IO uint32_t intfl;                                /*  0x0004          Interrupt Flags                                                              */
    __IO uint32_t inten;                                /*  0x0008          Interrupt Enable/Disable Controls                                            */
    __IO uint32_t svm_events;                           /*  0x000C          SVM Event Status Flags (read-only)                                           */
    __IO uint32_t wud_ctrl;                             /*  0x0010          Wake-Up Detect Control                                                       */
    __IO uint32_t wud_pulse0;                           /*  0x0014          WUD Pulse To Mode Bit 0                                                      */
    __IO uint32_t wud_pulse1;                           /*  0x0018          WUD Pulse To Mode Bit 1                                                      */
    __IO uint32_t wud_seen0;                            /*  0x001C          Wake-up Detect Status for P0/P1/P2/P3                                        */
    __IO uint32_t wud_seen1;                            /*  0x0020          Wake-up Detect Status for P4/P5/P6/P7                                        */
    __R  uint32_t rsv024[3];                            /*  0x0024-0x002C                                                                                */
    __IO uint32_t pt_regmap_ctrl;                       /*  0x0030          PT Register Mapping Control                                                  */
    __R  uint32_t rsv034;                               /*  0x0034                                                                                       */
    __IO uint32_t die_type;                             /*  0x0038          Die Type ID Register                                                         */
    __IO uint32_t base_part_num;                        /*  0x003C          Base Part Number                                                             */
    __IO uint32_t mask_id0;                             /*  0x0040          Mask ID Register 0                                                           */
    __IO uint32_t mask_id1;                             /*  0x0044          Mask ID Register 1                                                           */
    __IO uint32_t peripheral_reset;                     /*  0x0048          Peripheral Reset Control Register                                            */
} mxc_pwrman_regs_t;


/*
   Register offsets for module PWRMAN.
*/

#define MXC_R_PWRMAN_OFFS_PWR_RST_CTRL                      ((uint32_t)0x00000000UL)
#define MXC_R_PWRMAN_OFFS_INTFL                             ((uint32_t)0x00000004UL)
#define MXC_R_PWRMAN_OFFS_INTEN                             ((uint32_t)0x00000008UL)
#define MXC_R_PWRMAN_OFFS_SVM_EVENTS                        ((uint32_t)0x0000000CUL)
#define MXC_R_PWRMAN_OFFS_WUD_CTRL                          ((uint32_t)0x00000010UL)
#define MXC_R_PWRMAN_OFFS_WUD_PULSE0                        ((uint32_t)0x00000014UL)
#define MXC_R_PWRMAN_OFFS_WUD_PULSE1                        ((uint32_t)0x00000018UL)
#define MXC_R_PWRMAN_OFFS_WUD_SEEN0                         ((uint32_t)0x0000001CUL)
#define MXC_R_PWRMAN_OFFS_WUD_SEEN1                         ((uint32_t)0x00000020UL)
#define MXC_R_PWRMAN_OFFS_PT_REGMAP_CTRL                    ((uint32_t)0x00000030UL)
#define MXC_R_PWRMAN_OFFS_DIE_TYPE                          ((uint32_t)0x00000038UL)
#define MXC_R_PWRMAN_OFFS_BASE_PART_NUM                     ((uint32_t)0x0000003CUL)
#define MXC_R_PWRMAN_OFFS_MASK_ID0                          ((uint32_t)0x00000040UL)
#define MXC_R_PWRMAN_OFFS_MASK_ID1                          ((uint32_t)0x00000044UL)
#define MXC_R_PWRMAN_OFFS_PERIPHERAL_RESET                  ((uint32_t)0x00000048UL)


/*
   Field positions and masks for module PWRMAN.
*/

#define MXC_F_PWRMAN_PWR_RST_CTRL_AFE_POWERED_POS           2
#define MXC_F_PWRMAN_PWR_RST_CTRL_AFE_POWERED               ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_AFE_POWERED_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_IO_ACTIVE_POS             3
#define MXC_F_PWRMAN_PWR_RST_CTRL_IO_ACTIVE                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_IO_ACTIVE_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED_POS           4
#define MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED               ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_PULLUPS_ENABLED_POS       5
#define MXC_F_PWRMAN_PWR_RST_CTRL_PULLUPS_ENABLED           ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_PULLUPS_ENABLED_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_FIRMWARE_RESET_POS        8
#define MXC_F_PWRMAN_PWR_RST_CTRL_FIRMWARE_RESET            ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_FIRMWARE_RESET_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP_RESET_POS      9
#define MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP_RESET          ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP_RESET_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_TAMPER_DETECT_POS         16
#define MXC_F_PWRMAN_PWR_RST_CTRL_TAMPER_DETECT             ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_TAMPER_DETECT_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP_POS            17
#define MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_ARM_LOCKUP_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_ARM_POS        18
#define MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_ARM            ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_ARM_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_WATCHDOG_TIMEOUT_POS      19
#define MXC_F_PWRMAN_PWR_RST_CTRL_WATCHDOG_TIMEOUT          ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_WATCHDOG_TIMEOUT_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_SYSMAN_POS     20
#define MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_SYSMAN         ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_FW_COMMAND_SYSMAN_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_SRSTN_ASSERTION_POS       21
#define MXC_F_PWRMAN_PWR_RST_CTRL_SRSTN_ASSERTION           ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_SRSTN_ASSERTION_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_POR_POS                   22
#define MXC_F_PWRMAN_PWR_RST_CTRL_POR                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_POR_POS))
#define MXC_F_PWRMAN_PWR_RST_CTRL_LOW_POWER_MODE_POS        31
#define MXC_F_PWRMAN_PWR_RST_CTRL_LOW_POWER_MODE            ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PWR_RST_CTRL_LOW_POWER_MODE_POS))

#define MXC_F_PWRMAN_INTFL_V1_2_WARNING_POS                 0
#define MXC_F_PWRMAN_INTFL_V1_2_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_V1_2_WARNING_POS))
#define MXC_F_PWRMAN_INTFL_V1_8_WARNING_POS                 1
#define MXC_F_PWRMAN_INTFL_V1_8_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_V1_8_WARNING_POS))
#define MXC_F_PWRMAN_INTFL_RTC_WARNING_POS                  2
#define MXC_F_PWRMAN_INTFL_RTC_WARNING                      ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_RTC_WARNING_POS))
#define MXC_F_PWRMAN_INTFL_VDDA_WARNING_POS                 3
#define MXC_F_PWRMAN_INTFL_VDDA_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_VDDA_WARNING_POS))
#define MXC_F_PWRMAN_INTFL_VDDB_WARNING_POS                 4
#define MXC_F_PWRMAN_INTFL_VDDB_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_VDDB_WARNING_POS))
#define MXC_F_PWRMAN_INTFL_VDDIO_WARNING_POS                5
#define MXC_F_PWRMAN_INTFL_VDDIO_WARNING                    ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_VDDIO_WARNING_POS))
#define MXC_F_PWRMAN_INTFL_VDDIOH_WARNING_POS               6
#define MXC_F_PWRMAN_INTFL_VDDIOH_WARNING                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTFL_VDDIOH_WARNING_POS))

#define MXC_F_PWRMAN_INTEN_V1_2_WARNING_POS                 0
#define MXC_F_PWRMAN_INTEN_V1_2_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_V1_2_WARNING_POS))
#define MXC_F_PWRMAN_INTEN_V1_8_WARNING_POS                 1
#define MXC_F_PWRMAN_INTEN_V1_8_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_V1_8_WARNING_POS))
#define MXC_F_PWRMAN_INTEN_RTC_WARNING_POS                  2
#define MXC_F_PWRMAN_INTEN_RTC_WARNING                      ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_RTC_WARNING_POS))
#define MXC_F_PWRMAN_INTEN_VDDA_WARNING_POS                 3
#define MXC_F_PWRMAN_INTEN_VDDA_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_VDDA_WARNING_POS))
#define MXC_F_PWRMAN_INTEN_VDDB_WARNING_POS                 4
#define MXC_F_PWRMAN_INTEN_VDDB_WARNING                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_VDDB_WARNING_POS))
#define MXC_F_PWRMAN_INTEN_VDDIO_WARNING_POS                5
#define MXC_F_PWRMAN_INTEN_VDDIO_WARNING                    ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_VDDIO_WARNING_POS))
#define MXC_F_PWRMAN_INTEN_VDDIOH_WARNING_POS               6
#define MXC_F_PWRMAN_INTEN_VDDIOH_WARNING                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_INTEN_VDDIOH_WARNING_POS))

#define MXC_F_PWRMAN_SVM_EVENTS_V1_2_WARNING_POS            0
#define MXC_F_PWRMAN_SVM_EVENTS_V1_2_WARNING                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_V1_2_WARNING_POS))
#define MXC_F_PWRMAN_SVM_EVENTS_V1_8_WARNING_POS            1
#define MXC_F_PWRMAN_SVM_EVENTS_V1_8_WARNING                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_V1_8_WARNING_POS))
#define MXC_F_PWRMAN_SVM_EVENTS_RTC_WARNING_POS             2
#define MXC_F_PWRMAN_SVM_EVENTS_RTC_WARNING                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_RTC_WARNING_POS))
#define MXC_F_PWRMAN_SVM_EVENTS_VDDA_WARNING_POS            3
#define MXC_F_PWRMAN_SVM_EVENTS_VDDA_WARNING                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_VDDA_WARNING_POS))
#define MXC_F_PWRMAN_SVM_EVENTS_VDDB_WARNING_POS            4
#define MXC_F_PWRMAN_SVM_EVENTS_VDDB_WARNING                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_VDDB_WARNING_POS))
#define MXC_F_PWRMAN_SVM_EVENTS_VDDIO_WARNING_POS           5
#define MXC_F_PWRMAN_SVM_EVENTS_VDDIO_WARNING               ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_VDDIO_WARNING_POS))
#define MXC_F_PWRMAN_SVM_EVENTS_VDDIOH_WARNING_POS          6
#define MXC_F_PWRMAN_SVM_EVENTS_VDDIOH_WARNING              ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_SVM_EVENTS_VDDIOH_WARNING_POS))

#define MXC_F_PWRMAN_WUD_CTRL_PAD_SELECT_POS                0
#define MXC_F_PWRMAN_WUD_CTRL_PAD_SELECT                    ((uint32_t)(0x0000003FUL << MXC_F_PWRMAN_WUD_CTRL_PAD_SELECT_POS))
#define MXC_F_PWRMAN_WUD_CTRL_PAD_MODE_POS                  8
#define MXC_F_PWRMAN_WUD_CTRL_PAD_MODE                      ((uint32_t)(0x00000003UL << MXC_F_PWRMAN_WUD_CTRL_PAD_MODE_POS))
#define MXC_F_PWRMAN_WUD_CTRL_CLEAR_ALL_POS                 12
#define MXC_F_PWRMAN_WUD_CTRL_CLEAR_ALL                     ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_CTRL_CLEAR_ALL_POS))
#define MXC_F_PWRMAN_WUD_CTRL_CTRL_ENABLE_POS               16
#define MXC_F_PWRMAN_WUD_CTRL_CTRL_ENABLE                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_CTRL_CTRL_ENABLE_POS))

#define MXC_F_PWRMAN_WUD_SEEN0_GPIO0_POS                    0
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO0                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO0_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO1_POS                    1
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO1                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO1_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO2_POS                    2
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO2                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO2_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO3_POS                    3
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO3                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO3_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO4_POS                    4
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO4                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO4_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO5_POS                    5
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO5                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO5_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO6_POS                    6
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO6                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO6_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO7_POS                    7
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO7                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO7_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO8_POS                    8
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO8                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO8_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO9_POS                    9
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO9                        ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO9_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO10_POS                   10
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO10                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO10_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO11_POS                   11
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO11                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO11_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO12_POS                   12
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO12                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO12_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO13_POS                   13
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO13                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO13_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO14_POS                   14
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO14                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO14_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO15_POS                   15
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO15                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO15_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO16_POS                   16
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO16                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO16_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO17_POS                   17
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO17                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO17_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO18_POS                   18
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO18                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO18_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO19_POS                   19
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO19                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO19_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO20_POS                   20
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO20                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO20_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO21_POS                   21
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO21                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO21_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO22_POS                   22
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO22                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO22_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO23_POS                   23
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO23                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO23_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO24_POS                   24
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO24                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO24_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO25_POS                   25
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO25                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO25_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO26_POS                   26
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO26                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO26_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO27_POS                   27
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO27                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO27_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO28_POS                   28
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO28                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO28_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO29_POS                   29
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO29                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO29_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO30_POS                   30
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO30                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO30_POS))
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO31_POS                   31
#define MXC_F_PWRMAN_WUD_SEEN0_GPIO31                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN0_GPIO31_POS))

#define MXC_F_PWRMAN_WUD_SEEN1_GPIO32_POS                   0
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO32                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO32_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO33_POS                   1
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO33                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO33_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO34_POS                   2
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO34                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO34_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO35_POS                   3
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO35                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO35_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO36_POS                   4
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO36                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO36_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO37_POS                   5
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO37                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO37_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO38_POS                   6
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO38                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO38_POS))
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO39_POS                   7
#define MXC_F_PWRMAN_WUD_SEEN1_GPIO39                       ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_WUD_SEEN1_GPIO39_POS))

#define MXC_F_PWRMAN_PT_REGMAP_CTRL_ME02A_MODE_POS          0
#define MXC_F_PWRMAN_PT_REGMAP_CTRL_ME02A_MODE              ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PT_REGMAP_CTRL_ME02A_MODE_POS))

#define MXC_F_PWRMAN_BASE_PART_NUM_BASE_PART_NUMBER_POS     0
#define MXC_F_PWRMAN_BASE_PART_NUM_BASE_PART_NUMBER         ((uint32_t)(0x0000FFFFUL << MXC_F_PWRMAN_BASE_PART_NUM_BASE_PART_NUMBER_POS))

#define MXC_F_PWRMAN_MASK_ID0_REVISION_ID_POS               0
#define MXC_F_PWRMAN_MASK_ID0_REVISION_ID                   ((uint32_t)(0x0000000FUL << MXC_F_PWRMAN_MASK_ID0_REVISION_ID_POS))
#define MXC_F_PWRMAN_MASK_ID0_MASK_ID_POS                   4
#define MXC_F_PWRMAN_MASK_ID0_MASK_ID                       ((uint32_t)(0x0FFFFFFFUL << MXC_F_PWRMAN_MASK_ID0_MASK_ID_POS))

#define MXC_F_PWRMAN_MASK_ID1_MASK_ID_POS                   0
#define MXC_F_PWRMAN_MASK_ID1_MASK_ID                       ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRMAN_MASK_ID1_MASK_ID_POS))
#define MXC_F_PWRMAN_MASK_ID1_MASK_ID_ENABLE_POS            31
#define MXC_F_PWRMAN_MASK_ID1_MASK_ID_ENABLE                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_MASK_ID1_MASK_ID_ENABLE_POS))

#define MXC_F_PWRMAN_PERIPHERAL_RESET_SSB_POS               0
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SSB                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_SSB_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIX_POS              1
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIX                  ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_SPIX_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_PMU_POS               2
#define MXC_F_PWRMAN_PERIPHERAL_RESET_PMU                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_PMU_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_USB_POS               3
#define MXC_F_PWRMAN_PERIPHERAL_RESET_USB                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_USB_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_CRC_POS               4
#define MXC_F_PWRMAN_PERIPHERAL_RESET_CRC                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_CRC_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TPU_POS               5
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TPU                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TPU_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_WATCHDOG0_POS         6
#define MXC_F_PWRMAN_PERIPHERAL_RESET_WATCHDOG0             ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_WATCHDOG0_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_GPIO_POS              7
#define MXC_F_PWRMAN_PERIPHERAL_RESET_GPIO                  ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_GPIO_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER0_POS            8
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER0                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER0_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER1_POS            9
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER1                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER1_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER2_POS            10
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER2                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER2_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER3_POS            11
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER3                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER3_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER4_POS            12
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER4                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER4_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER5_POS            13
#define MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER5                ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_TIMER5_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_PULSE_TRAIN_POS       14
#define MXC_F_PWRMAN_PERIPHERAL_RESET_PULSE_TRAIN           ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_PULSE_TRAIN_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_UART0_POS             15
#define MXC_F_PWRMAN_PERIPHERAL_RESET_UART0                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_UART0_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_UART1_POS             16
#define MXC_F_PWRMAN_PERIPHERAL_RESET_UART1                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_UART1_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_UART2_POS             17
#define MXC_F_PWRMAN_PERIPHERAL_RESET_UART2                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_UART2_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_I2CM0_POS             19
#define MXC_F_PWRMAN_PERIPHERAL_RESET_I2CM0                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_I2CM0_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_I2CM1_POS             20
#define MXC_F_PWRMAN_PERIPHERAL_RESET_I2CM1                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_I2CM1_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_I2CS_POS              22
#define MXC_F_PWRMAN_PERIPHERAL_RESET_I2CS                  ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_I2CS_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM0_POS             23
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM0                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM0_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM1_POS             24
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM1                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM1_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM2_POS             25
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM2                 ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_SPIM2_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_OWM_POS               27
#define MXC_F_PWRMAN_PERIPHERAL_RESET_OWM                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_OWM_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_ADC_POS               28
#define MXC_F_PWRMAN_PERIPHERAL_RESET_ADC                   ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_ADC_POS))
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIS_POS              29
#define MXC_F_PWRMAN_PERIPHERAL_RESET_SPIS                  ((uint32_t)(0x00000001UL << MXC_F_PWRMAN_PERIPHERAL_RESET_SPIS_POS))



#ifdef __cplusplus
}
#endif

#endif   /* _MXC_PWRMAN_REGS_H_ */
