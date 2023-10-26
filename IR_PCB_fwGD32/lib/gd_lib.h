/*
 * gd_lib.h
 *
 *  Created on: 12 июл. 2023 г.
 *      Author: laurelindo
 */

#ifndef LIB_GD_LIB_H_
#define LIB_GD_LIB_H_

#include "types.h"
#include "gd32e11x_kl.h"
#include "core_cm4.h"
#include "core_cmFunc.h"
#include "board.h"
#include "kl_buf.h"

#if 1 // ============================ General ==================================
// ==== Build time ====
// Define symbol BUILD_TIME in main.cpp options with value ${current_date}.
// Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
#define STRINGIFY(x)    # x
#define XSTRINGIFY(x)   STRINGIFY(x)

// Virtual class for IRQ handlers and timer callbacks
class IrqHandler_t {
public:
    virtual void IIrqHandlerI() = 0;
};

// ==== Math ====
#define MIN_(a, b)   ( ((a)<(b))? (a) : (b) )
#define MAX_(a, b)   ( ((a)>(b))? (a) : (b) )
#define ABS(a)      ( ((a) < 0)? -(a) : (a) )

// IRQ priorities
#define IRQ_PRIO_LOW            15  // Minimum
#define IRQ_PRIO_MEDIUM         9
#define IRQ_PRIO_HIGH           7
#define IRQ_PRIO_VERYHIGH       4   // Higher than systick
#endif

// ========================== Simple delay ===============================
static inline void DelayLoop(volatile uint32_t ACounter) { while(ACounter--); }

namespace Random { // ======================== Random ==========================
int32_t do_rand(uint32_t *ctx);
int32_t rand();
// Generate pseudo-random value
int32_t Generate(int32_t LowInclusive, int32_t HighInclusive);
// Seed pseudo-random generator with new seed
void Seed(uint32_t Seed);

// True random
void TrueInit();
void TrueDeinit();
// Generate truly random value
uint32_t TrueGenerate(uint32_t LowInclusive, uint32_t HighInclusive);
// Seed pseudo random with true random
void SeedWithTrue();
} // namespace

// ================================ NVIC =======================================
#define NVIC_PRIORITY_MASK(prio) ((prio) << (8U - (unsigned)CORTEX_PRIORITY_BITS))

namespace Nvic {
    void EnableVector(IRQn_Type IrqN, uint32_t prio);
    void DisableVector(IRQn_Type IrqN);
    void SetSystemHandlerPriority(uint32_t handler, uint32_t prio);
    void ClearPending(IRQn_Type IrqN);
} // namespace


#if 1 // ========================== HW Timer ===================================


#endif

static inline void GetMcuSerialNum(uint32_t Ser[3]) {
    Ser[0] = *(volatile uint32_t *)(0x1FFFF7E8);
    Ser[1] = *(volatile uint32_t *)(0x1FFFF7EC);
    Ser[2] = *(volatile uint32_t *)(0x1FFFF7F0);
}

namespace Gpio { // ========================== GPIO ============================

enum PullUpDown_t { PullNone = 0b00, PullUp = 0b01, PullDown = 0b10 };
enum OutMode_t    { PushPull = 0, OpenDrain = 1 };
enum Speed_t      { speed10MHz=0b01UL, speed2MHz = 0b10UL, speed50MHz = 0b11UL, speed120MHz = 0b111 };
#define PIN_SPEED_DEFAULT   speed10MHz

// ==== Input state ====
__attribute__((__always_inline__))
static inline bool IsHi(GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->ISTAT & (1UL << APin); }
__attribute__((__always_inline__))
static inline bool IsHi(const GPIO_TypeDef *PGpio, uint32_t APin) { return PGpio->ISTAT & (1UL << APin); }
__attribute__((__always_inline__))
static inline bool IsLo(GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->ISTAT & (1UL << APin)); }
__attribute__((__always_inline__))
static inline bool IsLo(const GPIO_TypeDef *PGpio, uint32_t APin) { return !(PGpio->ISTAT & (1UL << APin)); }

// ==== SetHi, SetLo, Toggle ====
__attribute__((__always_inline__))
static inline void SetHi(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BOP = 1UL << APin; }
__attribute__((__always_inline__))
static inline void SetLo(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->BC = 1UL << APin;  }
__attribute__((__always_inline__))
static inline void Toggle(GPIO_TypeDef *PGpio, uint32_t APin) { PGpio->OCTL ^= 1UL << APin; }

// ==== Setup ====
void SetupOut(GPIO_TypeDef *PGpio, const uint32_t PinN,
        const OutMode_t OutMode, const Speed_t ASpeed = PIN_SPEED_DEFAULT);

void SetupInput(GPIO_TypeDef *PGpio, const uint32_t PinN, const PullUpDown_t PullUpDown);

void SetupAnalog(GPIO_TypeDef *PGpio, const uint32_t PinN);

void SetupAlterFunc(GPIO_TypeDef *PGpio, const uint32_t PinN,
        const OutMode_t OutMode, const Speed_t ASpeed = PIN_SPEED_DEFAULT);

} // namespace Gpio

class Pin_t {
public:
    Pin_t() : PGpio(nullptr), PinN(0) {}
    Pin_t(GPIO_TypeDef *APGpio, uint32_t APinN) : PGpio(APGpio), PinN(APinN) {}
    GPIO_TypeDef *PGpio;
    uint32_t PinN;
    inline void SetHi() { Gpio::SetHi(PGpio, PinN); }
    inline void SetLo() { Gpio::SetLo(PGpio, PinN); }
    void SetupOut(const Gpio::OutMode_t OutMode, const Gpio::Speed_t ASpeed = Gpio::speed10MHz) {
        Gpio::SetupOut(PGpio, PinN, OutMode, ASpeed);
    }
};

// ==== PWM output ====
/* Example:
 * #define LED_R_PIN { GPIOB, 1, TIM3, 4, invInverted, omPushPull, 255 }
 * PinOutputPWM_t Led {LedPin};
*/

struct PwmSetup_t {
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    TIM_TypeDef *PTimer;
    uint32_t TimerChnl;
    Inverted_t Inverted;
    Gpio::OutMode_t OutputType;
    uint32_t TopValue;
    PwmSetup_t(GPIO_TypeDef *APGpio, uint16_t APin,
            TIM_TypeDef *APTimer, uint32_t ATimerChnl,
            Inverted_t AInverted, Gpio::OutMode_t AOutputType,
            uint32_t ATopValue) : PGpio(APGpio), Pin(APin), PTimer(APTimer),
                    TimerChnl(ATimerChnl), Inverted(AInverted), OutputType(AOutputType),
                    TopValue(ATopValue) {}
};

#if 1 // =========================== I2C =======================================
class Shell_t;
class Thread_t;

class i2c_t {
private:
    I2C_TypeDef *pi2c;
    GPIO_TypeDef *PGpioScl, *PGpioSda;
    uint32_t PinScl, PinSda;
    retv IBusyWait(uint32_t Timeout_ms = 4);
    retv IWaitStartSent();
    retv IWaitAddrSent();
    retv IWaitDataSent();
    void IReset();
    void IWakeup();
    void OnTransmissionEnd(retv Rslt);
    void DisableAndClearIRQs();
    // Transmission context
    uint8_t IAddr = 0;
    Buf_t IBufW, IBufR;
    uint32_t ILen2 = 0;
    Thread_t* PThd = nullptr;
#if I2C_USE_SEMAPHORE
    binary_semaphore_t BSemaphore;
#endif
public:
    i2c_t(I2C_TypeDef *pi2c,
            GPIO_TypeDef *PGpioScl, uint32_t PinScl,
            GPIO_TypeDef *PGpioSda, uint32_t PinSda) :
        pi2c(pi2c), PGpioScl(PGpioScl), PGpioSda(PGpioSda), PinScl(PinScl), PinSda(PinSda) {}
    void Init();
    void Standby();
    void Resume();
    void PutBusLow();

    void ScanBus(Shell_t *PShell);
    retv CheckBusAndResume();
    retv CheckAddress(uint32_t Addr);
    retv Write    (uint8_t Addr, uint8_t *WPtr,  uint32_t WLength, uint32_t Timeout_ms = 999);
    retv WriteRead(uint8_t Addr, uint8_t *WPtr,  uint32_t WLength, uint8_t *RPtr, uint32_t RLength, uint32_t Timeout_ms = 999);
    retv Read     (uint8_t Addr, uint8_t *RPtr, uint32_t RLength, uint32_t Timeout_ms = 999);
    retv WriteBytes(uint8_t Addr, uint32_t ByteCnt, ...);

//    retv WriteWrite(uint32_t Addr, uint8_t *WPtr1, uint32_t WLength1, uint8_t *WPtr2, uint32_t WLength2);
    // Inner use
    void IProcessIRQ();
};

extern i2c_t i2c0, i2c1;
#endif

#if 1 // =========================== DMA =======================================
class DMA_t {
private:
    DMAChannel_t *PChnl;
    uint32_t ChnlN; // Required for IRQ flags cleanup
public:
    DMA_t(DMAChannel_t *APChnl,
            ftVoidPVoidW32 PIrqFunc = nullptr,
            void *PIrqParam = nullptr,
            uint32_t AIrqPrio = IRQ_PRIO_MEDIUM);
    void Init() const;
    void Init(volatile void* PeriphAddr, void* MemAddr, uint32_t AMode, uint16_t Cnt) const;
    void SetPeriphAddr(volatile void* Addr) const { PChnl->CPADDR  = (uint32_t)Addr; }
    void SetMemoryAddr(void* Addr)          const { PChnl->CMADDR  = (uint32_t)Addr; }
    void* GetMemoryAddr()                   const { return (void*)PChnl->CMADDR; }
    void SetMode(uint32_t AMode)            const { PChnl->CTL = AMode; }
    void SetTransferDataCnt(uint16_t Cnt)   const { PChnl->CNT = Cnt; }
    uint16_t GetTransferDataCnt()           const { return PChnl->CNT; }

    void Enable()                           const { PChnl->CTL |=  DMA_CHNL_EN; }
    void Disable()                          const { PChnl->CTL &= ~DMA_CHNL_EN; }
    void ClearIrq() const;
    void DisableAndClearIRQ() const;
};
#endif // DMA

#if 1 // ============================== SPI ====================================
class Spi_t {
public:
    SPI_TypeDef *PSpi;
    Spi_t(SPI_TypeDef *ASpi) : PSpi(ASpi) {}
    enum class cpha {FirstEdge, SecondEdge};
    enum class cpol {IdleLow, IdleHigh};
    enum class BitNumber {n8, n16};
    // Example: boMSB, cpolIdleLow, cphaFirstEdge, sbFdiv2, bitn8
    void Setup(BitOrder BitOrdr, cpol CPOL, cpha CPHA,
            int32_t Bitrate_Hz, BitNumber BitNum = BitNumber::n8) const;

    void Enable()    { PSpi->Enable(); }
    void Disable()   { PSpi->Disable(); }
    void SetRxOnly()     { PSpi->CTL0 |=  SPI_CTL0_RO; }
    void SetFullDuplex() { PSpi->CTL0 &= ~SPI_CTL0_RO; }

    // DMA
    void EnTxDma()    { PSpi->CTL1 |=  SPI_CTL1_DMATEN; }
    void DisTxDma()   { PSpi->CTL1 &= ~SPI_CTL1_DMATEN; }
    void EnRxDma()    { PSpi->CTL1 |=  SPI_CTL1_DMAREN; }
    void DisRxDma()   { PSpi->CTL1 &= ~SPI_CTL1_DMAREN; }
    void EnRxTxDma()  { PSpi->CTL1 |=  (SPI_CTL1_DMATEN | SPI_CTL1_DMAREN); }
    void DisRxTxDma() { PSpi->CTL1 &= ~(SPI_CTL1_DMATEN | SPI_CTL1_DMAREN); }

    // IRQ
    void EnNvicIrq(const uint32_t Priority) const {
        if     (PSpi == SPI0) Nvic::EnableVector(SPI0_IRQn, Priority);
        else if(PSpi == SPI1) Nvic::EnableVector(SPI1_IRQn, Priority);
        else if(PSpi == SPI2) Nvic::EnableVector(SPI2_IRQn, Priority);
    }
    void DisNvicIrq() const {
        if     (PSpi == SPI0) Nvic::DisableVector(SPI0_IRQn);
        else if(PSpi == SPI1) Nvic::DisableVector(SPI1_IRQn);
        else if(PSpi == SPI2) Nvic::DisableVector(SPI2_IRQn);
    }
//    void SetupRxIrqCallback(ftVoidVoid AIrqHandler) const;

    void ClearRxBuf() { while(PSpi->STAT & SPI_STAT_RBNE) (void)PSpi->DATA; }

    void Write(uint32_t AData) {
        PSpi->WaitForTBEHi();
        PSpi->DATA = AData;
    }

    uint16_t WriteRead(uint32_t AData) {
        PSpi->DATA = AData;
        PSpi->WaitForRBNEHi(); // Wait for SPI transmission to complete
        return PSpi->DATA;
    }

    void EnQuadRead();
    void EnQuadWrite();
    void DisQuad();

//    void Transmit(uint8_t Params, uint8_t *ptr, uint32_t Len) const {
//        PSpi->Disable();
////        PSpi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
////        if(Params & 0x80) PSpi->CR1 |= SPI_CR1_LSBFIRST; // 0 = MSB, 1 = LSB
////        if(Params & 0x40) PSpi->CR1 |= SPI_CR1_CPOL;     // 0 = IdleLow, 1 = IdleHigh
////        if(Params & 0x20) PSpi->CR1 |= SPI_CR1_CPHA;     // 0 = FirstEdge, 1 = SecondEdge
////        PSpi->CR1 |= (Params & 0x07) << 3; // Setup divider
////        PSpi->CR2 = ((uint16_t)0b0111 << 8);
////        (void)PSpi->SR; // Read Status reg to clear some flags
//        // Do it
//        PSpi->Enable();
//        while(Len) {
//            PSpi->Write(*ptr);
//            PSpi->WaitRBNEHi(); // Wait for SPI transmission to complete
//            *ptr = PSpi->Read();
//            ptr++;
//            Len--;
//        }
//    }

};
#endif

namespace Clk { // ======================== Clocking ===========================
/* PLL: input [1; 25] MHz, typ 8MHz; output [16; 120] MHz
 */

// Frequency values
extern uint32_t AHBFreqHz, APB1FreqHz, APB2FreqHz;

void SetPllMulti(uint32_t Multi);

enum class CkSysSrc { CK_IRC8M = 0b00, CK_XTAL = 0b01, CK_PLL = 0b10 };

void UpdateFreqValues();
void PrintFreqs();
uint32_t GetTimInputFreq(const uint32_t TimerN);
uint32_t GetTimInputFreq(const TIM_TypeDef *PTimer);

} // namespace

class PinOutputPWM_t {
private:
    const PwmSetup_t ISetup;
public:
    void Set(uint32_t AValue) const { ISetup.PTimer->SetChnlValue(ISetup.TimerChnl, AValue); }
//    uint32_t Get() const { return *TMR_PCCR(ITmr, ISetup.TimerChnl); }
    void Init() const;
//    void Deinit() const { Timer_t::Deinit(); PinSetupAnalog(ISetup.PGpio, ISetup.Pin); }
    void SetFrequencyHz(uint32_t FreqHz) const { // Set freq changing prescaler
        // Figure out input timer freq
        uint32_t UpdFreqMax = Clk::GetTimInputFreq(ISetup.PTimer) / (ISetup.PTimer->CAR + 1);
        uint32_t Psc = UpdFreqMax / FreqHz;
        if(Psc != 0) Psc--;
    //    Printf("InputFreq=%u; UpdFreqMax=%u; div=%u; ARR=%u\r", InputFreq, UpdFreqMax, div, ITmr->ARR);
        ISetup.PTimer->PSC = Psc;
        ISetup.PTimer->CNT = 0;  // Reset counter to start from scratch
        ISetup.PTimer->GenerateUpdateEvt();
    }
    void SetTopValue(uint32_t Value) const { ISetup.PTimer->SetTopValue(Value); }
//    void SetTmrClkFreq(uint32_t FreqHz) const { Timer_t::SetTmrClkFreq(FreqHz); }
//    void SetPrescaler(uint32_t PrescalerValue) const { Timer_t::SetPrescaler(PrescalerValue); }
    PinOutputPWM_t(const PwmSetup_t &ASetup) : ISetup(ASetup) {}
//    PinOutputPWM_t(GPIO_TypeDef *PGpio, uint16_t Pin,
//            TIM_TypeDef *PTimer, uint32_t TimerChnl,
//            Inverted_t Inverted, Gpio::OutMode_t OutputType, uint32_t TopValue) :
//                PGpio(PGpio), Pin(Pin), ITmr(PTimer), TimerChnl(TimerChnl),
//                Inverted(Inverted), OutputType(OutputType), TopValue(TopValue) {}
};

#endif /* LIB_GD_LIB_H_ */
