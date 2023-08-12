/*
 * board.h
 *
 *  Created on: 01.02.2017
 *      Author: Kreyl
 */

#pragma once

// ==== General ====
#define BOARD_NAME          "IR_PCB1"
#define APP_NAME            "IR_PCB"

// MCU type as defined in the ST header.
#define STM32F072xB     // no matter, 8 or B

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ         12000000

// Ch timer
#define STM32_ST_IRQ_PRIORITY   2
#define STM32_ST_USE_TIMER      14
#define SYS_TIM_CLK             (Clk.APBFreqHz)

#define SIMPLESENSORS_ENABLED   TRUE
#define BUTTONS_ENABLED         FALSE
#define ADC_REQUIRED            FALSE
#define I2C1_ENABLED            FALSE
#define I2C_USE_SEMAPHORE       FALSE

// Flash
#define FLASH_START_ADDR        0x08000000UL
#define FW_BOOT_SZ              0x2000UL // 8192
#define FW_START_ADDR           (FLASH_START_ADDR + FW_BOOT_SZ)
#define FW_MAX_SZ               0xD000UL // 53248, about half of (128 total - 2 settings - 8 bootloader)
#define FW_PARAMS_ADDR          (FW_START_ADDR + FW_MAX_SZ)
#define FW_PARAMS_SZ            2048UL // PageSz
#define FW_STORAGE_ADDR         (FW_PARAMS_ADDR + FW_PARAMS_SZ)
#define SETTINGS_STORAGE_ADDR   0x0801F800UL // 128k - 2k, where 2k is Flash Page size

#if 1 // ========================== GPIO =======================================
// PortMinTim_t: GPIO, Pin, Tim, TimChnl, invInverted, omPushPull, TopValue

// Inputs
#define INPUT1          GPIOB, 3
#define INPUT2          GPIOB, 4
#define INPUT3          GPIOB, 5

// Outputs
#define OUTPUT1         GPIOB, 13
#define OUTPUT2         GPIOB, 14
#define OUTPUT3         GPIOB, 15

// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     9
#define UART_RX_PIN     10

// Green LED
#define LUMOS_PIN       { GPIOA, 0, TIM2, 1, invNotInverted, omPushPull, 255 }

// Front LEDs
#define LED_FRONT1      { GPIOA, 1, TIM2, 2, invNotInverted, omPushPull, 255 }
#define LED_FRONT2      { GPIOA, 2, TIM2, 3, invNotInverted, omPushPull, 255 }
#define FRONT_LEDS_CNT  2

// Side LEDs
#define LED_PWM1        { GPIOC, 6, TIM3, 1, invInverted, omPushPull, 255 }
#define LED_PWM2        { GPIOC, 7, TIM3, 2, invInverted, omPushPull, 255 }
#define LED_PWM3        { GPIOC, 8, TIM3, 3, invInverted, omPushPull, 255 }
#define LED_PWM4        { GPIOC, 9, TIM3, 4, invInverted, omPushPull, 255 }
#define SIDE_LEDS_CNT   4

// IR Rcvr
#define IR_WKUP         GPIOC, 13
#define IR_DATA         GPIOB, 1 // TIM14 & IRQ
#define IR_PWR          GPIOB, 2

#endif // GPIO

#if 1 // =========================== DMA =======================================
#define STM32_DMA_REQUIRED  TRUE
// ==== Uart ====
#define UART_DMA_TX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_LOW | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_TCIE)
#define UART_DMA_RX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_MEDIUM | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CIRC)
#define UART_DMA_TX     STM32_DMA_STREAM_ID(1, 2)
#define UART_DMA_RX     STM32_DMA_STREAM_ID(1, 3)
#define UART_DMA_CHNL   0   // Dummy

#endif // DMA

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN FALSE
#define UART_TXBUF_SZ   1024
#define UART_RXBUF_SZ   256
#define CMD_BUF_SZ      256

#define CMD_UART        USART1


#define CMD_UART_PARAMS \
    CMD_UART, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN, \
    UART_DMA_TX, UART_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // independent clock

#endif
