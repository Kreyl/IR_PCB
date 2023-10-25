/*
 * gd_uart.cpp
 *
 *  Created on: 14 июл. 2023 г.
 *      Author: laurelindo
 */

#include "gd_uart.h"
#include "gd_lib.h"
#include "yartos.h"

#if 1 // ==== TX DMA IRQ ====
// Wrapper for TX IRQ
void UartDmaTxIrqHandler(void *p, uint32_t flags) { ((BaseUart_t*)p)->IRQDmaTxHandler(); }

void BaseUart_t::IRQDmaTxHandler() {
    DmaTx.Disable(); // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SZ)) PRead = TXBuf; // Circulate pointer
    if(IFullSlotsCount == 0) ITxDmaIsIdle = true; // Nothing left to send
    else ISendViaDMA();
}
#endif

#if 1 // ==== TX / RX ====
void BaseUart_t::ISendViaDMA() {
    uint32_t PartSz = (TXBuf + UART_TXBUF_SZ) - PRead; // Cnt from PRead to end of buf
    ITransSize = MIN_(IFullSlotsCount, PartSz);
    if(ITransSize != 0) {
        ITxDmaIsIdle = false;
        DmaTx.SetMemoryAddr(PRead);
        DmaTx.SetTransferDataCnt(ITransSize);
        Params->Uart->STAT0 &= ~USART_STAT0_TC; // Clear TC flag
        DmaTx.Enable();
    }
}

retv BaseUart_t::IPutByteNow(uint8_t b) {
    while(!(Params->Uart->STAT0 & USART_STAT0_TBE));
    Params->Uart->DATA = b;
    while(!(Params->Uart->STAT0 & USART_STAT0_TBE));
    return retv::Ok;
}

retv BaseUart_t::IPutByte(uint8_t b) {
    if(IFullSlotsCount >= UART_TXBUF_SZ) return retv::Overflow;
    *PWrite++ = b;
    if(PWrite >= &TXBuf[UART_TXBUF_SZ]) PWrite = TXBuf;   // Circulate buffer
    IFullSlotsCount++;
    return retv::Ok;
}

void BaseUart_t::IStartTransmissionIfNotYet() {
    if(ITxDmaIsIdle) ISendViaDMA();
}

retv BaseUart_t::GetByte(uint8_t *b) {
    int32_t WIndx = UART_RXBUF_SZ - DmaRx.GetTransferDataCnt();
    int32_t BytesCnt = WIndx - RIndx;
    if(BytesCnt < 0) BytesCnt += UART_RXBUF_SZ;
    if(BytesCnt == 0) return retv::Empty;
    *b = IRxBuf[RIndx++];
    if(RIndx >= UART_RXBUF_SZ) RIndx = 0;
    return retv::Ok;
}
#endif

void BaseUart_t::Init() {
    RCU->EnUart(Params->Uart); // Clock
    OnClkChange();  // Setup baudrate

    // ==== TX ====
    Gpio::SetupAlterFunc(Params->PGpioTx, Params->PinTx, Gpio::PushPull, Gpio::speed50MHz);
    DmaTx.Init();
    DmaTx.SetPeriphAddr(&Params->Uart->DATA);
    DmaTx.SetMode(Params->DmaModeTx);
    ITxDmaIsIdle = true;

    // ==== RX ====
    Gpio::SetupInput(Params->PGpioRx, Params->PinRx, Gpio::PullUp);
    DmaRx.Init(&Params->Uart->DATA, IRxBuf, Params->DmaModeRx, UART_RXBUF_SZ);
    DmaRx.Enable();

    // UART Regs setup
    Params->Uart->CTL0 = USART_CTL0_TEN | USART_CTL0_REN;     // TX & RX en, 8bit, no parity
    Params->Uart->CTL1 = 0;  // Nothing interesting there
    Params->Uart->CTL2 = USART_CTL2_DENT | USART_CTL2_DENR; // Enable DMA at TX & RX
    Params->Uart->CTL0 |= USART_CTL0_UEN;    // Enable USART
}

void BaseUart_t::OnClkChange() {
    if(Params->Uart == USART0) Params->Uart->BAUD = Clk::APB2FreqHz / Params->Baudrate; // The only UART on APB2
    else                       Params->Uart->BAUD = Clk::APB1FreqHz / Params->Baudrate; // Others are on APB1
}

void BaseUart_t::StopTx() {
    Params->Uart->CTL0 &= ~USART_CTL0_UEN; // Disable UART
    DmaTx.DisableAndClearIRQ();
    ITxDmaIsIdle = true;
    PRead = TXBuf;
    PWrite = TXBuf;
    Params->Uart->CTL0 |= USART_CTL0_UEN;
}
