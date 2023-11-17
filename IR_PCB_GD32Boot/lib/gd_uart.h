#ifndef LIB_GD_UART_H_
#define LIB_GD_UART_H_

#include <inttypes.h>
#include "gd32e11x_kl.h"
#include "shell.h"

struct UartParams_t {
    uint32_t Baudrate;
    USART_TypeDef* Uart;
    GPIO_TypeDef *PGpioTx;
    uint16_t PinTx;
    GPIO_TypeDef *PGpioRx;
    uint16_t PinRx;
    // DMA
    DMAChannel_t *DmaChnlTx, *DmaChnlRx;

    UartParams_t(uint32_t ABaudrate, USART_TypeDef* AUart,
            GPIO_TypeDef *APGpioTx, uint16_t APinTx,
            GPIO_TypeDef *APGpioRx, uint16_t APinRx,
            DMAChannel_t *ADmaChnlTx, DMAChannel_t *ADmaChnlRx
    ) : Baudrate(ABaudrate), Uart(AUart),
            PGpioTx(APGpioTx), PinTx(APinTx), PGpioRx(APGpioRx), PinRx(APinRx),
            DmaChnlTx(ADmaChnlTx), DmaChnlRx(ADmaChnlRx) {}
};

void UartDmaTxIrqHandler(void *p, uint32_t flags);

class BaseUart_t {
protected:
    const UartParams_t* const Params;
    char TXBuf[UART_TXBUF_SZ];
    char *PRead, *PWrite;
    bool ITxDmaIsIdle;
    uint32_t IFullSlotsCount, ITransSize;
    void ISendViaDMA();
    int32_t OldWIndx, RIndx;
    uint8_t IRxBuf[UART_RXBUF_SZ];
    DMA_t DmaTx, DmaRx;
    retv IPutByte(uint8_t b);
    retv IPutByteNow(uint8_t b);
    void IStartTransmissionIfNotYet();
    // ==== Constructor ====
    BaseUart_t(const UartParams_t &APParams) : Params(&APParams)
    , PRead(TXBuf), PWrite(TXBuf), ITxDmaIsIdle(true), IFullSlotsCount(0), ITransSize(0),
        OldWIndx(0), RIndx(0),
        DmaTx(Params->DmaChnlTx, UartDmaTxIrqHandler, this, IRQ_PRIO_LOW),
        DmaRx(Params->DmaChnlRx, nullptr, nullptr, IRQ_PRIO_LOW) {}
    retv GetByte(uint8_t *b);
public:
    void Init();
    void Shutdown();
    void OnClkChange();
    void StopTx();
    // Enable/Disable
    void EnableTx()  { Params->Uart->EnableTx();  }
    void DisableTx() { Params->Uart->DisableTx(); }
    void EnableRx()  { Params->Uart->EnableRx();  }
    void DisableRx() { Params->Uart->DisableRx(); }
    // Inner use
    void IRQDmaTxHandler();
};

class CmdUart_t : public BaseUart_t, public Shell_t {
private:
    retv IPutChar(char c) { return IPutByte(c); }
    void IStartTransmissionIfNotYet() { BaseUart_t::IStartTransmissionIfNotYet(); }
public:
    CmdUart_t(const UartParams_t &APParams) : BaseUart_t(APParams) {}
    retv TryParseRxBuff() {
        uint8_t b;
        while(GetByte(&b) == retv::Ok) {
            if(Cmd.PutChar(b) == pdrNewCmd) return retv::Ok;
        } // while get byte
        return retv::Fail;
    }
    void PrintfNow(const char *S) { while(*S) IPutByteNow(*S++); }

    retv ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {return retv::Ok;}
    retv TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {return retv::Ok;}
};

#endif /* LIB_GD_UART_H_ */
