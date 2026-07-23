/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)
#define CPUCLK_FREQ                                                     80000000



/* Defines for carPWM */
#define carPWM_INST                                                        TIMA0
#define carPWM_INST_IRQHandler                                  TIMA0_IRQHandler
#define carPWM_INST_INT_IRQN                                    (TIMA0_INT_IRQn)
#define carPWM_INST_CLK_FREQ                                             2000000
/* GPIO defines for channel 0 */
#define GPIO_carPWM_C0_PORT                                                GPIOA
#define GPIO_carPWM_C0_PIN                                         DL_GPIO_PIN_0
#define GPIO_carPWM_C0_IOMUX                                      (IOMUX_PINCM1)
#define GPIO_carPWM_C0_IOMUX_FUNC                     IOMUX_PINCM1_PF_TIMA0_CCP0
#define GPIO_carPWM_C0_IDX                                   DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_carPWM_C1_PORT                                                GPIOA
#define GPIO_carPWM_C1_PIN                                         DL_GPIO_PIN_1
#define GPIO_carPWM_C1_IOMUX                                      (IOMUX_PINCM2)
#define GPIO_carPWM_C1_IOMUX_FUNC                     IOMUX_PINCM2_PF_TIMA0_CCP1
#define GPIO_carPWM_C1_IDX                                   DL_TIMER_CC_1_INDEX
/* Defines for car1N */
#define car1N_INST                                                         TIMG6
#define car1N_INST_IRQHandler                                   TIMG6_IRQHandler
#define car1N_INST_INT_IRQN                                     (TIMG6_INT_IRQn)
#define car1N_INST_CLK_FREQ                                              2000000
/* GPIO defines for channel 1 */
#define GPIO_car1N_C1_PORT                                                 GPIOA
#define GPIO_car1N_C1_PIN                                         DL_GPIO_PIN_22
#define GPIO_car1N_C1_IOMUX                                      (IOMUX_PINCM47)
#define GPIO_car1N_C1_IOMUX_FUNC                     IOMUX_PINCM47_PF_TIMG6_CCP1
#define GPIO_car1N_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for car2N */
#define car2N_INST                                                        TIMG12
#define car2N_INST_IRQHandler                                  TIMG12_IRQHandler
#define car2N_INST_INT_IRQN                                    (TIMG12_INT_IRQn)
#define car2N_INST_CLK_FREQ                                             10000000
/* GPIO defines for channel 0 */
#define GPIO_car2N_C0_PORT                                                 GPIOB
#define GPIO_car2N_C0_PIN                                         DL_GPIO_PIN_13
#define GPIO_car2N_C0_IOMUX                                      (IOMUX_PINCM30)
#define GPIO_car2N_C0_IOMUX_FUNC                    IOMUX_PINCM30_PF_TIMG12_CCP0
#define GPIO_car2N_C0_IDX                                    DL_TIMER_CC_0_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (1499U)


/* Defines for MPU6050 hardware I2C: PA28=SDA, PA31=SCL */
#define MPU_I2C_INST                                                        I2C0
#define MPU_I2C_INST_IRQHandler                                  I2C0_IRQHandler
#define MPU_I2C_INST_INT_IRQN                                      I2C0_INT_IRQn
#define MPU_I2C_BUS_SPEED_HZ                                              400000
#define GPIO_MPU_I2C_SDA_PORT                                              GPIOA
#define GPIO_MPU_I2C_SDA_PIN                                      DL_GPIO_PIN_28
#define GPIO_MPU_I2C_IOMUX_SDA                                    (IOMUX_PINCM3)
#define GPIO_MPU_I2C_IOMUX_SDA_FUNC                     IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_MPU_I2C_SCL_PORT                                              GPIOA
#define GPIO_MPU_I2C_SCL_PIN                                      DL_GPIO_PIN_31
#define GPIO_MPU_I2C_IOMUX_SCL                                    (IOMUX_PINCM6)
#define GPIO_MPU_I2C_IOMUX_SCL_FUNC                     IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for ESP */
#define ESP_INST                                                           UART1
#define ESP_INST_FREQUENCY                                              40000000
#define ESP_INST_IRQHandler                                     UART1_IRQHandler
#define ESP_INST_INT_IRQN                                         UART1_INT_IRQn
#define GPIO_ESP_RX_PORT                                                   GPIOA
#define GPIO_ESP_TX_PORT                                                   GPIOA
#define GPIO_ESP_RX_PIN                                           DL_GPIO_PIN_18
#define GPIO_ESP_TX_PIN                                           DL_GPIO_PIN_17
#define GPIO_ESP_IOMUX_RX                                        (IOMUX_PINCM40)
#define GPIO_ESP_IOMUX_TX                                        (IOMUX_PINCM39)
#define GPIO_ESP_IOMUX_RX_FUNC                         IOMUX_PINCM40_PF_UART1_RX
#define GPIO_ESP_IOMUX_TX_FUNC                         IOMUX_PINCM39_PF_UART1_TX
#define ESP_BAUD_RATE                                                   (115200)
#define ESP_IBRD_40_MHZ_115200_BAUD                                         (21)
#define ESP_FBRD_40_MHZ_115200_BAUD                                         (45)
/* Defines for ADC */
#define ADC_INST                                                            ADC0
#define ADC_INST_IRQHandler                                      ADC0_IRQHandler
#define ADC_INST_INT_IRQN                                        (ADC0_INT_IRQn)
#define ADC_ADCMEM_0                                          DL_ADC12_MEM_IDX_0
#define ADC_ADCMEM_0_REF                         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_ADCMEM_0_REF_VOLTAGE_V                                           3.3
#define ADC_ADCMEM_1                                          DL_ADC12_MEM_IDX_1
#define ADC_ADCMEM_1_REF                         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_ADCMEM_1_REF_VOLTAGE_V                                           3.3
#define ADC_ADCMEM_2                                          DL_ADC12_MEM_IDX_2
#define ADC_ADCMEM_2_REF                         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_ADCMEM_2_REF_VOLTAGE_V                                           3.3
#define ADC_ADCMEM_3                                          DL_ADC12_MEM_IDX_3
#define ADC_ADCMEM_3_REF                         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_ADCMEM_3_REF_VOLTAGE_V                                           3.3
#define ADC_ADCMEM_4                                          DL_ADC12_MEM_IDX_4
#define ADC_ADCMEM_4_REF                         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_ADCMEM_4_REF_VOLTAGE_V                                           3.3
#define ADC_ADCMEM_5                                          DL_ADC12_MEM_IDX_5
#define ADC_ADCMEM_5_REF                         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_ADCMEM_5_REF_VOLTAGE_V                                           3.3
#define GPIO_ADC_C0_PORT                                                   GPIOA
#define GPIO_ADC_C0_PIN                                           DL_GPIO_PIN_27
#define GPIO_ADC_C1_PORT                                                   GPIOA
#define GPIO_ADC_C1_PIN                                           DL_GPIO_PIN_26
#define GPIO_ADC_C2_PORT                                                   GPIOA
#define GPIO_ADC_C2_PIN                                           DL_GPIO_PIN_25
#define GPIO_ADC_C3_PORT                                                   GPIOA
#define GPIO_ADC_C3_PIN                                           DL_GPIO_PIN_24
#define GPIO_ADC_C4_PORT                                                   GPIOB
#define GPIO_ADC_C4_PIN                                           DL_GPIO_PIN_25
#define GPIO_ADC_C5_PORT                                                   GPIOB
#define GPIO_ADC_C5_PIN                                           DL_GPIO_PIN_24

/* Defines for ccdADC */
#define ccdADC_INST                                                         ADC1
#define ccdADC_INST_IRQHandler                                   ADC1_IRQHandler
#define ccdADC_INST_INT_IRQN                                     (ADC1_INT_IRQn)
#define ccdADC_ADCMEM_0                                       DL_ADC12_MEM_IDX_0
#define ccdADC_ADCMEM_0_REF                      DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ccdADC_ADCMEM_0_REF_VOLTAGE_V                                        3.3
#define GPIO_ccdADC_C0_PORT                                                GPIOA
#define GPIO_ccdADC_C0_PIN                                        DL_GPIO_PIN_15



/* Port definition for Pin Group encoder */
#define encoder_PORT                                                     (GPIOB)

/* Defines for RightA: GPIOB.14 with pinCMx 31 on package pin 2 */
// groups represented: ["myShow","encoder"]
// pins affected: ["Buzzer","RightA","RightB"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define encoder_RightA_IIDX                                 (DL_GPIO_IIDX_DIO14)
#define encoder_RightA_PIN                                      (DL_GPIO_PIN_14)
#define encoder_RightA_IOMUX                                     (IOMUX_PINCM31)
/* Defines for RightB: GPIOB.15 with pinCMx 32 on package pin 3 */
#define encoder_RightB_IIDX                                 (DL_GPIO_IIDX_DIO15)
#define encoder_RightB_PIN                                      (DL_GPIO_PIN_15)
#define encoder_RightB_IOMUX                                     (IOMUX_PINCM32)
/* Port definition for Pin Group mosSW */
#define mosSW_PORT                                                       (GPIOA)

/* Defines for mos1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define mosSW_mos1_PIN                                          (DL_GPIO_PIN_14)
#define mosSW_mos1_IOMUX                                         (IOMUX_PINCM36)
/* Defines for mos2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define mosSW_mos2_PIN                                          (DL_GPIO_PIN_16)
#define mosSW_mos2_IOMUX                                         (IOMUX_PINCM38)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for KEY1: GPIOB.17 with pinCMx 43 on package pin 14 */
#define KEY_KEY1_PIN                                            (DL_GPIO_PIN_17)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM43)
/* Defines for KEY2: GPIOB.18 with pinCMx 44 on package pin 15 */
#define KEY_KEY2_PIN                                            (DL_GPIO_PIN_18)
#define KEY_KEY2_IOMUX                                           (IOMUX_PINCM44)
/* Defines for KEY3: GPIOB.19 with pinCMx 45 on package pin 16 */
#define KEY_KEY3_PIN                                            (DL_GPIO_PIN_19)
#define KEY_KEY3_IOMUX                                           (IOMUX_PINCM45)
/* Port definition for Pin Group myShow */
#define myShow_PORT                                                      (GPIOB)

/* Defines for LED: GPIOB.22 with pinCMx 50 on package pin 21 */
#define myShow_LED_PIN                                          (DL_GPIO_PIN_22)
#define myShow_LED_IOMUX                                         (IOMUX_PINCM50)
/* Defines for Buzzer: GPIOB.21 with pinCMx 49 on package pin 20 */
#define myShow_Buzzer_IIDX                                  (DL_GPIO_IIDX_DIO21)
#define myShow_Buzzer_PIN                                       (DL_GPIO_PIN_21)
#define myShow_Buzzer_IOMUX                                      (IOMUX_PINCM49)
/* Defines for ioLeft: GPIOA.23 with pinCMx 53 on package pin 24 */
#define infra_ioLeft_PORT                                                (GPIOA)
#define infra_ioLeft_PIN                                        (DL_GPIO_PIN_23)
#define infra_ioLeft_IOMUX                                       (IOMUX_PINCM53)
/* Defines for ioRight: GPIOB.27 with pinCMx 58 on package pin 29 */
#define infra_ioRight_PORT                                               (GPIOB)
#define infra_ioRight_PIN                                       (DL_GPIO_PIN_27)
#define infra_ioRight_IOMUX                                      (IOMUX_PINCM58)
/* Port definition for Pin Group encoder2 */
#define encoder2_PORT                                                    (GPIOA)

/* Defines for leftA: GPIOA.2 with pinCMx 7 on package pin 42 */
// pins affected by this interrupt request:["leftA","leftB"]
#define encoder2_INT_IRQN                                       (GPIOA_INT_IRQn)
#define encoder2_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define encoder2_leftA_IIDX                                  (DL_GPIO_IIDX_DIO2)
#define encoder2_leftA_PIN                                       (DL_GPIO_PIN_2)
#define encoder2_leftA_IOMUX                                      (IOMUX_PINCM7)
/* Defines for leftB: GPIOA.8 with pinCMx 19 on package pin 54 */
#define encoder2_leftB_IIDX                                  (DL_GPIO_IIDX_DIO8)
#define encoder2_leftB_PIN                                       (DL_GPIO_PIN_8)
#define encoder2_leftB_IOMUX                                     (IOMUX_PINCM19)


/* CANFD0 is retained for later use; the PWM-only test does not initialize it. */
#define MCAN0_INST                                                        CANFD0
#define GPIO_MCAN0_CAN_TX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_TX_PIN                                     DL_GPIO_PIN_12
#define GPIO_MCAN0_IOMUX_CAN_TX                                  (IOMUX_PINCM34)
#define GPIO_MCAN0_IOMUX_CAN_TX_FUNC               IOMUX_PINCM34_PF_CANFD0_CANTX
#define GPIO_MCAN0_CAN_RX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_RX_PIN                                     DL_GPIO_PIN_13
#define GPIO_MCAN0_IOMUX_CAN_RX                                  (IOMUX_PINCM35)
#define GPIO_MCAN0_IOMUX_CAN_RX_FUNC               IOMUX_PINCM35_PF_CANFD0_CANRX

#define MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_STD_ID_FILTER_NUM          (1)
#define MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR     (48)
#define MCAN0_INST_MCAN_EXT_ID_FILTER_NUM          (1)
#define MCAN0_INST_MCAN_TX_BUFF_START_ADDR         (148)
#define MCAN0_INST_MCAN_TX_BUFF_SIZE               (2)
#define MCAN0_INST_MCAN_FIFO_1_START_ADDR          (192)
#define MCAN0_INST_MCAN_FIFO_1_NUM                 (2)
#define MCAN0_INST_MCAN_TX_EVENT_START_ADDR        (164)
#define MCAN0_INST_MCAN_TX_EVENT_SIZE              (2)
#define MCAN0_INST_MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
#define MCAN0_INST_MCAN_RX_BUFF_START_ADDR         (208)
#define MCAN0_INST_MCAN_FIFO_0_START_ADDR          (172)
#define MCAN0_INST_MCAN_FIFO_0_NUM                 (3)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_motorOutputsLow(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_carPWM_init(void);
void SYSCFG_DL_car1N_init(void);
void SYSCFG_DL_car2N_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_MPU_I2C_init(void);
void SYSCFG_DL_ESP_init(void);
void SYSCFG_DL_ADC_init(void);
void SYSCFG_DL_ccdADC_init(void);
void SYSCFG_DL_MCAN0_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
