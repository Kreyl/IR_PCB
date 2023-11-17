/*
 * board.h
 *
 *  Created on: 28.08.2023
 *      Author: Kreyl
 */

#ifndef BOARD_H__
#define BOARD_H__

// ==== General ====
#define BOARD_NAME          "IR_PCBv3"
#define APP_NAME            "IRPCB_Bootloader"

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ             12000000UL

#if 1 // ==== MCU info ====
// Configuration of the Cortex-M4 processor and core peripherals
#define __CM4_REV                   0x0001   // Core revision r0p1
#define __MPU_PRESENT               0        // GD32E11x do not provide MPU
#define CORTEX_SIMPLIFIED_PRIORITY  FALSE
#define CORTEX_PRIORITY_BITS        4        // GD32E11x uses 4 bits for the priority levels
#define __FPU_PRESENT               1        // FPU present

#define CORTEX_USE_FPU              __FPU_PRESENT
#define CORTEX_MODEL                4
#endif

#if 1 // ==== OS timer settings ====
#define SYS_TIM                     TIM10
#define SYS_TIM_IRQn                TIMER0_TRG_CMT_TIMER10_IRQn
#define SYS_TIM_IRQ_HANDLER         TIMER0_TRG_CMT_TIMER10_IRQHandler
#define SYS_TIM_IRQ_PRIORITY        2
#endif

// ==== Internal FLASH ====
// Setup this
#define FILENAME_PATTERN        "Fw*.bin"
#define BOOTLOADER_RSRVD_SPACE  0x5000UL    // 20480 bytes for bootloader
// Do not touch
#define APP_START_ADDR          (FLASH_BASE + BOOTLOADER_RSRVD_SPACE)


#if 1 // ========================== GPIO =======================================
// EXTI
#define INDIVIDUAL_EXTI_IRQ_REQUIRED    FALSE

// User GPIOs
#define Gpio1           PB2
#define Gpio2           PB10
#define Gpio3           PB11
#define Gpio4           PA8

// UART
#define UART_TX_PIN     PA2
#define UART_RX_PIN     PA3

// Green LED
#define LUMOS_PIN       { PA10, TIM0, 2, invNotInverted, Gpio::PushPull, 255 }

// Spi Flash
#define FLASH_NSS       PA15
#define FLASH_SCK       PB3
#define FLASH_MISO      PB4
#define FLASH_MOSI      PB5
#define FLASH_IO2       PB6
#define FLASH_IO3       PB7

#endif // GPIO

#if 1 // ==== USB ====
#define USB_SOF_CB_EN       FALSE // SOF callback not used
#define USB_IRQ_PRIO        14
#define USB_TXBUF_CNT       4   // 4 buffers of size=EP_BULK_SZ each
// Crystalless mode: IRC48M utilized, it syncs using SOF pulses at PA8. Therefore, PA8 is occupied.
#define USB_CRYSTALLESS_EN  TRUE
// Next options: set to 1U if enabled, 0U if disabled. Required for Setup Request reply.
#define USB_REMOTE_WKUP_EN  0U
#define USB_SELF_POWERED    0U  // if not self powered, then it is bus powered
// Hardware depended, do not touch
#define USB_SET_ADDRESS_ACK_BY_HW   FALSE
#define USB_SEQUENCE_WORKAROUND     TRUE
#define USB_SET_ADDR_AFTER_ZEROPKT  FALSE
#define USB_NUM_EP_MAX              3U // Excluding Ep0
#define USB_IRQ_HANDLER             USBFS_IRQHandler // 0x14C
#define USB_IRQ_NUMBER              USBFS_IRQn // 67

#endif

#if 1 // =========================== DMA =======================================
// SPI FLASH
#define SPIFLASH_DMA_RX     DMA0_Channel1
#define SPIFLASH_DMA_TX     DMA0_Channel2
// Uart
#define UART_DMA_RX         DMA0_Channel5
#define UART_DMA_TX         DMA0_Channel6

#endif // DMA

#if 1 // ========================== USART ======================================
//#define PRINTF_FLOAT_EN TRUE
#define UART_TXBUF_SZ   1024
#define UART_RXBUF_SZ   128
#define CMD_BUF_SZ      128
#define UART_RX_POLL_MS 99

#define CMD_UART        USART1

#define CMD_UART_PARAMS CMD_UART, UART_TX_PIN, UART_RX_PIN, UART_DMA_TX, UART_DMA_RX

#endif

#endif //BOARD_H__
