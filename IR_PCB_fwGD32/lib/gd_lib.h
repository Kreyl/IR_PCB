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
#include <vector>

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
#define IS_LIKE(v, precise, deviation)  (((precise - deviation) < v) and (v < (precise + deviation)))

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
class HwTim {
protected:
    TIM_TypeDef* itmr;
public:
    enum class TriggerIn {ITI0=0x00, ITI1=0x10, ITI2=0x20, ITI3=0x30, CI0F_ED=0x40,
        CI0FE0=0x50, CI1FE1=0x60, ETIFP=0x70};
    enum class MasterMode {Reset=0x00, Enable=0x10, Update=0x20, CaptureComparePulse0=0x30,
        Compare0=0x40, Compare1=0x50, Compare2=0x60, Compare3=0x70};
    enum class SlaveMode {Disable=0, QDecoder0=1, QDecoder1=2, QDecoder2=3,
        Restart=4, Pause=5, Event=6, ExtClk=7};
    enum class InputPsc {Div1=0UL, Div2=01UL, Div4=2UL, Div8=3UL};
    enum class ChnlMode {Output=0UL, ITS=3UL,
        CI0FE0=1UL, CI1FE0=2UL, CI1FE1=1UL, CI0FE1=2UL, CI2FE2=1UL, CI3FE2=2UL, CI3FE3=1UL, CI2FE3=2UL};
    enum class CmpMode { Timing=0b000UL, SetOoutput=0b001UL, ClearOutput=0b010UL,
        ToggleOnMatch=0b011UL, ForceLo=0b100UL, ForceHi=0b101UL, PWM0HiLo=0b110UL,
        PWM1LoHi=0b111};

    HwTim(TIM_TypeDef *APTimer) : itmr(APTimer) {}
    void Init()    const { RCU->EnTimer(itmr);  }
    void Deinit()  const { RCU->DisTimer(itmr); }
    void Enable()  const { itmr->Enable(); }
    void Disable() const { itmr->Disable(); }
    void SetInputFreqChangingPrescaler(uint32_t FreqHz) const;

    void SetUpdateFreqChangingPrescaler(uint32_t FreqHz) const;
    void SetUpdateFreqChangingTopValue(uint32_t FreqHz) const;
    void SetUpdateFreqChangingBoth(uint32_t FreqHz) const;
    void SetTopValue(uint32_t Value) const { itmr->SetTopValue(Value); }
    uint32_t GetTopValue() const { return itmr->GetTopValue(); }
    void EnableAutoreloadBuffering()  const { itmr->CTL0 |=  TIM_CTL0_ARSE; }
    void DisableAutoreloadBuffering() const { itmr->CTL0 &= ~TIM_CTL0_ARSE; }
    void SetPrescaler(uint32_t PrescalerValue) const { itmr->SetPrescaler(PrescalerValue); }
    uint32_t GetPrescaler() const { return itmr->GetPrescaler(); }
    void SetCounter(uint32_t Value) const { itmr->CNT = Value; }
    uint32_t GetCounter() const { return itmr->CNT; }

    // Compare
    void SetChnlValue(uint32_t ChnlN, uint32_t AValue) const {
        *((uint32_t*)(&itmr->CH0CV) + ChnlN) = AValue;
    }
    void SetChnl0Value(uint32_t AValue) const { itmr->CH0CV = AValue; }
    void SetChnl1Value(uint32_t AValue) const { itmr->CH1CV = AValue; }
    void SetChnl2Value(uint32_t AValue) const { itmr->CH2CV = AValue; }
    void SetChnl3Value(uint32_t AValue) const { itmr->CH3CV = AValue; }

    uint32_t GetChnl0Value() const { return itmr->CH0CV; }
    uint32_t GetChnl1Value() const { return itmr->CH1CV; }
    uint32_t GetChnl2Value() const { return itmr->CH2CV; }
    uint32_t GetChnl3Value() const { return itmr->CH3CV; }

    // Master/Slave
    void SetTriggerInput(TriggerIn TrgInput) const { itmr->SMCFG = (itmr->SMCFG & ~(0b111UL << 4)) | (uint32_t)TrgInput; }
    void SetEtrPolarity(Inverted_t AInverted) {
        if(AInverted == invInverted) itmr->SMCFG |= TIM_SMCFG_ETP;
        else itmr->SMCFG &= ~TIM_SMCFG_ETP;
    }
    void SelectMasterMode(MasterMode Mode) const { itmr->CTL1 = (itmr->CTL1 & ~(0b111UL << 4)) | (uint32_t)Mode; }
    void SelectSlaveMode(SlaveMode Mode) const { itmr->SMCFG = (itmr->SMCFG & ~0b111UL) | (uint32_t)Mode; }

    // Channels setup
    void EnPrimaryOutput() const { itmr->CCHP = 0xC000; }
    void SetChnlMode(uint32_t ChnlN, ChnlMode Mode) const {
        if     (ChnlN == 0) SET_BITS(itmr->CHCTL0, 0b11UL, (uint32_t)Mode, 0);
        else if(ChnlN == 1) SET_BITS(itmr->CHCTL0, 0b11UL, (uint32_t)Mode, 8);
        else if(ChnlN == 2) SET_BITS(itmr->CHCTL1, 0b11UL, (uint32_t)Mode, 0);
        else if(ChnlN == 3) SET_BITS(itmr->CHCTL1, 0b11UL, (uint32_t)Mode, 8);
    }
    void SetInputPsc(uint32_t ChnlN, InputPsc Psc) const {
        if     (ChnlN == 0) SET_BITS(itmr->CHCTL0, 0b11UL, (uint32_t)Psc, 2);
        else if(ChnlN == 1) SET_BITS(itmr->CHCTL0, 0b11UL, (uint32_t)Psc, 10);
        else if(ChnlN == 2) SET_BITS(itmr->CHCTL1, 0b11UL, (uint32_t)Psc, 2);
        else if(ChnlN == 3) SET_BITS(itmr->CHCTL1, 0b11UL, (uint32_t)Psc, 10);
    }
    void SetInputActiveEdge(uint32_t InputN, RiseFall Rsfll) const {
        uint32_t bits = (Rsfll == RiseFall::Rising)? 0b0000UL : (Rsfll == RiseFall::Falling)? 0b0010UL : 0b1010L;
        if     (InputN == 0) SET_BITS(itmr->CHCTL2, 0b1010UL, bits, 0);  // CI0FE0 and CI1FE0
        else if(InputN == 1) SET_BITS(itmr->CHCTL2, 0b1010UL, bits, 4);  // CI1FE1 and CI0FE1
        else if(InputN == 2) SET_BITS(itmr->CHCTL2, 0b1010UL, bits, 8);  // CI2FE2 and CI3FE2
        else if(InputN == 3) SET_BITS(itmr->CHCTL2, 0b1010UL, bits, 12); // CI3FE3 and CI2FE3
    }
    void SetOutputCmpMode(uint32_t ChnlN, CmpMode Mode) const {
        if     (ChnlN == 0) SET_BITS(itmr->CHCTL0, 0b111UL, (uint32_t)Mode, 4);
        else if(ChnlN == 1) SET_BITS(itmr->CHCTL0, 0b111UL, (uint32_t)Mode, 12);
        else if(ChnlN == 2) SET_BITS(itmr->CHCTL1, 0b111UL, (uint32_t)Mode, 4);
        else if(ChnlN == 3) SET_BITS(itmr->CHCTL1, 0b111UL, (uint32_t)Mode, 12);
    }
    void EnableOutputShadow(uint32_t ChnlN) const {
        if     (ChnlN == 0) itmr->CHCTL0 |= 1UL << 3;
        else if(ChnlN == 1) itmr->CHCTL0 |= 1UL << 11;
        else if(ChnlN == 2) itmr->CHCTL1 |= 1UL << 3;
        else if(ChnlN == 3) itmr->CHCTL1 |= 1UL << 11;
    }

    void EnChnl(uint32_t ChnlN) const {
        if     (ChnlN == 0) itmr->CHCTL2 |= 1UL << 0;
        else if(ChnlN == 1) itmr->CHCTL2 |= 1UL << 4;
        else if(ChnlN == 2) itmr->CHCTL2 |= 1UL << 8;
        else if(ChnlN == 3) itmr->CHCTL2 |= 1UL << 12;
    }

    // DMA
    volatile uint32_t* GetChnlRegAddr(uint32_t ChnlN) const {
        if     (ChnlN == 0) return &itmr->CH0CV;
        else if(ChnlN == 1) return &itmr->CH1CV;
        else if(ChnlN == 2) return &itmr->CH2CV;
        else                return &itmr->CH3CV;
    }
    void EnableDmaOnTrigger() const { itmr->DMAINTEN |= TIM_DMAINTEN_TRGDEN; }
    void EnableDmaOnCapture(uint32_t ChnlN) const { itmr->DMAINTEN |= TIM_DMAINTEN_DMAEN(ChnlN); }
    void EnableDmaOnUpdate()  const { itmr->DMAINTEN |= TIM_DMAINTEN_UPDEN; }
    // Evt
    void GenerateUpdateEvt()  const { itmr->GenerateUpdateEvt(); }
    // Enable IRQ
    void EnableIrqOnUpdate()  const  { itmr->DMAINTEN |= TIM_DMAINTEN_UPIE; }
    void EnableIrqOnCompare0() const { itmr->DMAINTEN |= TIM_DMAINTEN_CH0IE; }
    void EnableIrqOnCompare1() const { itmr->DMAINTEN |= TIM_DMAINTEN_CH1IE; }
    void EnableIrqOnCompare2() const { itmr->DMAINTEN |= TIM_DMAINTEN_CH2IE; }
    void EnableIrqOnCompare3() const { itmr->DMAINTEN |= TIM_DMAINTEN_CH3IE; }
    // Disable IRQ
    void DisableIrqOnUpdate()   const { itmr->DMAINTEN &= ~TIM_DMAINTEN_UPIE; }
    void DisableIrqOnCompare0() const { itmr->DMAINTEN &= ~TIM_DMAINTEN_CH0IE; }
    void DisableIrqOnCompare1() const { itmr->DMAINTEN &= ~TIM_DMAINTEN_CH1IE; }
    void DisableIrqOnCompare2() const { itmr->DMAINTEN &= ~TIM_DMAINTEN_CH2IE; }
    void DisableIrqOnCompare3() const { itmr->DMAINTEN &= ~TIM_DMAINTEN_CH3IE; }
    // Clear IRQ
    void ClearUpdateIrqPendingBit()   const { itmr->INTF &= ~TIM_INTF_UPIF; }
    void ClearCompare0IrqPendingBit() const { itmr->INTF &= ~TIM_INTF_CH0IF; }
    void ClearCompare1IrqPendingBit() const { itmr->INTF &= ~TIM_INTF_CH1IF; }
    void ClearCompare2IrqPendingBit() const { itmr->INTF &= ~TIM_INTF_CH2IF; }
    void ClearCompare3IrqPendingBit() const { itmr->INTF &= ~TIM_INTF_CH3IF; }
    // Check
    bool IsEnabled()            const { return (itmr->CTL0 & TIM_CTL0_CEN); }
    bool IsUpdateIrqFired()     const { return (itmr->INTF & TIM_INTF_UPIF); }
    bool IsCompare0IrqFired()   const { return (itmr->INTF & TIM_INTF_CH0IF); }
    bool IsCompare1IrqFired()   const { return (itmr->INTF & TIM_INTF_CH1IF); }
    bool IsCompare2IrqFired()   const { return (itmr->INTF & TIM_INTF_CH2IF); }
    bool IsCompare3IrqFired()   const { return (itmr->INTF & TIM_INTF_CH3IF); }
    bool IsCompare0IrqEnabled() const { return (itmr->DMAINTEN & TIM_DMAINTEN_CH0IE); }
    bool IsCompare1IrqEnabled() const { return (itmr->DMAINTEN & TIM_DMAINTEN_CH1IE); }
    bool IsCompare2IrqEnabled() const { return (itmr->DMAINTEN & TIM_DMAINTEN_CH2IE); }
    bool IsCompare3IrqEnabled() const { return (itmr->DMAINTEN & TIM_DMAINTEN_CH3IE); }
};
#endif

static inline void GetMcuSerialNum(uint32_t Ser[3]) {
    Ser[0] = *(volatile uint32_t *)(0x1FFFF7E8);
    Ser[1] = *(volatile uint32_t *)(0x1FFFF7EC);
    Ser[2] = *(volatile uint32_t *)(0x1FFFF7F0);
}

namespace Gpio { // ========================== GPIO ============================

enum PullUpDown { PullNone = 0b00, PullUp = 0b01, PullDown = 0b10 };
enum OutMode    { PushPull = 0, OpenDrain = 1 };
enum Speed      { speed10MHz=0b01UL, speed2MHz = 0b10UL, speed50MHz = 0b11UL, speed120MHz = 0b111 };
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
__attribute__((__always_inline__))
static inline void Set(GPIO_TypeDef *PGpio, uint32_t APin, uint32_t Lvl) {
    if(Lvl == 0) PGpio->BC = 1UL << APin;
    else PGpio->BOP = 1UL << APin;
}

// ==== Setup ====
void SetupOut(GPIO_TypeDef *PGpio, const uint32_t PinN,
        const OutMode OutMode, const Speed ASpeed = PIN_SPEED_DEFAULT);

void SetupInput(GPIO_TypeDef *PGpio, const uint32_t PinN, const PullUpDown PullUpDown);

void SetupAnalog(GPIO_TypeDef *PGpio, const uint32_t PinN);

void SetupAlterFunc(GPIO_TypeDef *PGpio, const uint32_t PinN,
        const OutMode OutMode, const Speed ASpeed = PIN_SPEED_DEFAULT);

} // namespace Gpio

class Pin_t {
public:
    GPIO_TypeDef *pgpio;
    uint32_t pin_n;
    Pin_t() : pgpio(nullptr), pin_n(0) {}
    Pin_t(GPIO_TypeDef *APGpio, uint32_t APinN) : pgpio(APGpio), pin_n(APinN) {}
    inline void SetHi() { Gpio::SetHi(pgpio, pin_n); }
    inline void SetLo() { Gpio::SetLo(pgpio, pin_n); }
    inline void Set(uint32_t lvl) { Gpio::Set(pgpio, pin_n, lvl); }
    void SetupOut(const Gpio::OutMode OutMode, const Gpio::Speed ASpeed = Gpio::speed10MHz) {
        Gpio::SetupOut(pgpio, pin_n, OutMode, ASpeed);
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
    Gpio::OutMode OutputType;
    uint32_t TopValue;
    PwmSetup_t(GPIO_TypeDef *APGpio, uint16_t APin,
            TIM_TypeDef *APTimer, uint32_t ATimerChnl,
            Inverted_t AInverted, Gpio::OutMode AOutputType,
            uint32_t ATopValue) : PGpio(APGpio), Pin(APin), PTimer(APTimer),
                    TimerChnl(ATimerChnl), Inverted(AInverted), OutputType(AOutputType),
                    TopValue(ATopValue) {}
};

#if 1 // =========================== External IRQ ==============================
enum ExtiTrigType_t {ttRising, ttFalling, ttRisingFalling};

/* Make your class descendant of IrqHandler_t:
class cc1101_t : public IrqHandler_t {
    const PinIrq_t IGdo0;
    void IIrqHandler() { ... }
    cc1101_t(...) ... IGdo0(APGpio, AGdo0, pudNone, this), ...
}
    ...IGdo0.Init(ttFalling);
    ...IGdo0.EnableIrq(IRQ_PRIO_HIGH);
*/

// Pin to IRQ channel
#define PIN2IRQ_CHNL(Pin)   (IRQn_Type)(((Pin) > 9)? EXTI10_15_IRQn : (((Pin) > 4)? EXTI5_9_IRQn : ((Pin) + EXTI0_IRQn)))

// IRQ handlers
extern "C" {
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
extern IrqHandler_t *ExtiIrqHandler[16];
#else
extern ftVoidVoid ExtiIrqHandler[5], ExtiIrqHandler_9_5, ExtiIrqHandler_15_10;
#endif // INDIVIDUAL_EXTI_IRQ_REQUIRED
}

class PinIrq {
public:
    GPIO_TypeDef *pgpio;
    uint16_t pin_n;
    Gpio::PullUpDown pull_up_down;
    PinIrq(GPIO_TypeDef *APGpio, uint16_t APinN, Gpio::PullUpDown APullUpDown, ftVoidVoid PIrqHandler) :
        pgpio(APGpio), pin_n(APinN), pull_up_down(APullUpDown) {
#if INDIVIDUAL_EXTI_IRQ_REQUIRED
        ExtiIrqHandler[APinN] = PIrqHandler;
#else
        if(APinN >= 0 and APinN <= 4) ExtiIrqHandler[APinN] = PIrqHandler;
        else if(APinN <= 9) ExtiIrqHandler_9_5 = PIrqHandler;
        else ExtiIrqHandler_15_10 = PIrqHandler;
#endif // INDIVIDUAL_EXTI_IRQ_REQUIRED
    }

    bool IsHi() const { return Gpio::IsHi(pgpio, pin_n); }

    void SetTriggerType(ExtiTrigType_t ATriggerType) const {
        uint32_t IrqMsk = 1 << pin_n;
        switch(ATriggerType) {
            case ttRising:
                EXTI->RTEN |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTEN &= ~IrqMsk;  // Falling trigger disabled
                break;
            case ttFalling:
                EXTI->RTEN &= ~IrqMsk;  // Rising trigger disabled
                EXTI->FTEN |=  IrqMsk;  // Falling trigger enabled
                break;
            case ttRisingFalling:
                EXTI->RTEN |=  IrqMsk;  // Rising trigger enabled
                EXTI->FTEN |=  IrqMsk;  // Falling trigger enabled
                break;
        } // switch
    }

    // ttRising, ttFalling, ttRisingFalling
    void Init(ExtiTrigType_t ATriggerType) const {
        // Init pin as input
        Gpio::SetupInput(pgpio, pin_n, pull_up_down);
        // Connect EXTI line to the pin of the port
        uint32_t Indx   = pin_n / 4;            // Indx of EXTICR register
        uint32_t Offset = (pin_n & 0x03UL) * 4; // Offset in EXTICR register
        // Clear bits
        AFIO->EXTISS[Indx] &= ~(0b1111UL << Offset);
        // GPIOA requires all zeroes
        if     (pgpio == GPIOB) AFIO->EXTISS[Indx] |= 1UL << Offset;
        else if(pgpio == GPIOC) AFIO->EXTISS[Indx] |= 2UL << Offset;
        else if(pgpio == GPIOD) AFIO->EXTISS[Indx] |= 3UL << Offset;
        else if(pgpio == GPIOE) AFIO->EXTISS[Indx] |= 4UL << Offset;
        // Configure EXTI line
        uint32_t IrqMsk = 1 << pin_n;
        EXTI->INTEN  |=  IrqMsk;      // Interrupt mode enabled
        EXTI->EVEN  &= ~IrqMsk;      // Event mode disabled
        SetTriggerType(ATriggerType);
        EXTI->PD     =  IrqMsk;      // Clean irq flag
    }
    void EnableIrq(const uint32_t Priority) const { Nvic::EnableVector(PIN2IRQ_CHNL(pin_n), Priority); }
    void DisableIrq() const { Nvic::DisableVector(PIN2IRQ_CHNL(pin_n)); }
    void CleanIrqFlag() const { EXTI->PD = (1 << pin_n); }
    bool IsIrqPending() const { return EXTI->PD & (1 << pin_n); }
    void GenerateIrq()  const { EXTI->SWIEV = (1 << pin_n); }
};
#endif // EXTI

#if 1 // ============================== Watchdog ===============================
namespace Watchdog {
// Up to 32000 ms
void InitAndStart(uint32_t ms);

static inline void Reload() { FWDGT->CTL = 0xAAAA; }

void DisableInDebug();

}; // Namespace
#endif

#if 1 // =========================== I2C =======================================
class Shell;
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

    void ScanBus(Shell *PShell);
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
    void Init(volatile void* PeriphAddr, uint32_t AMode) const;
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

#if ADC_REQUIRED // ========================= ADC ==============================
namespace Adc {

struct Channel_t {
    GPIO_TypeDef *GPIO;
    uint32_t Pin;
    uint32_t ChannelN;
    Channel_t(uint32_t AChnl) : GPIO(nullptr), Pin(0), ChannelN(AChnl) {}
    Channel_t(GPIO_TypeDef *AGPIO, uint32_t APin, uint32_t AChnl) :
        GPIO(AGPIO), Pin(APin), ChannelN(AChnl) {}
};


struct Params {
    AdcPsc AdcClkPrescaler; // ADC clock must be within [0.1; 40] MHz
    AdcSampleTime SampleTime;
    AdcOversamplingRatio OversamplingRatio;
    AdcOversamplingShift OversamplingShift;
    ftVoidVoid DoneCallbackI;
    std::vector<Channel_t> Channels;
};

void Init(const Params& Setup);
void StartMeasurement();
uint32_t GetResult(uint32_t AChannel);
uint32_t Adc2mV(uint32_t AdcChValue, uint32_t VrefValue);

};
#endif

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

    void Enable()        { PSpi->Enable(); }
    void Disable()       { PSpi->Disable(); }
    void SetRxOnly()     { PSpi->CTL0 |=  SPI_CTL0_RO; }
    void SetFullDuplex() { PSpi->CTL0 &= ~SPI_CTL0_RO; }

    // DMA
    void EnTxDma()    { PSpi->EnTxDma(); }
    void DisTxDma()   { PSpi->DisTxDma(); }
    void EnRxDma()    { PSpi->EnRxDma(); }
    void DisRxDma()   { PSpi->DisRxDma(); }
    void EnRxTxDma()  { PSpi->EnRxTxDma(); }
    void DisRxTxDma() { PSpi->DisRxTxDma(); }

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

// =========================== Flash and Option bytes ==========================
namespace Flash {

void UnlockFlash();
void LockFlash();
void ClearPendingFlags();
retv WaitForLastOperation(uint32_t Timeout_ms);
retv ErasePage(uint32_t PageAddress);
retv ProgramWord(uint32_t Address, uint32_t Data);
retv ProgramBuf(uint32_t *ptr, uint32_t ByteSz, uint32_t Addr);
bool FirmwareIsLocked();
void LockFirmware();
void UnlockFirmware();

} // Namespace

namespace Clk { // ======================== Clocking ===========================
/* PLL: input [1; 25] MHz, typ 8MHz; output [16; 120] MHz */
// Frequency values
extern uint32_t AHBFreqHz, APB1FreqHz, APB2FreqHz;

// Enables
static inline void EnIRC40K() {
    RCU->RSTSCK |= RCU_RSTSCK_IRC40KEN;
    while(!(RCU->RSTSCK & RCU_RSTSCK_IRC40KSTB));
}

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
