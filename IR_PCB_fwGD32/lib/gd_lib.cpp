/*
 * gd_lib.cpp
 *
 *  Created on: 12 июл. 2023 г.
 *      Author: laurelindo
 */

#include "gd_lib.h"
#include "core_cm4.h"
#include "shell.h"

// Universal callback to run IrqHandler by timer
void VTmrUniversalCb(void *p) {
    Sys::LockFromIRQ();
    ((IrqHandler_t*)p)->IIrqHandlerI();
    Sys::UnlockFromIRQ();
}

namespace Random { // ======================== Random ==========================
static uint32_t next = 1;

int32_t do_rand(uint32_t *ctx) {
#if 0
    if(*ctx == 0) *ctx = 123459876;
    int32_t hi = *ctx / 127773;
    int32_t lo = *ctx % 127773;
    int32_t x = 16807 * lo - 2836 * hi;
    if(x < 0) x += 0x7FFFFFFF;
    return ((*ctx = x) % ((uint32_t)0x7fffffff + 1));
#else
    return ((*ctx = *ctx * 1103515245 + 12345) % ((uint32_t)0x7fffffff + 1));
#endif
}

int32_t rand() { return do_rand(&next); }

// Generate pseudo-random value
int32_t Generate(int32_t LowInclusive, int32_t HighInclusive) {
    uint32_t last = rand();
    return (last % (HighInclusive + 1 - LowInclusive)) + LowInclusive;
}
// Seed pseudo-random generator with new seed
void Seed(uint32_t Seed) { next = Seed; }

} // namespace


namespace Gpio { // ========================== GPIO ============================

void SetupOut(GPIO_TypeDef *PGpio, const uint32_t PinN, const OutMode_t OutMode, const Speed_t ASpeed) {
    RCU->EnGpio(PGpio);
    uint32_t CtlMode = ((uint32_t)OutMode << 2) | ((uint32_t)ASpeed & 0b11UL);
    PGpio->SetCtlMode(PinN, CtlMode);
    if(ASpeed == speed120MHz) PGpio->SPD |= 1UL << PinN;
    else PGpio->SPD &= ~(1UL << PinN);
}

void SetupInput(GPIO_TypeDef *PGpio, const uint32_t PinN, const PullUpDown_t PullUpDown) {
    RCU->EnGpio(PGpio);
    if(PullUpDown == PullNone) PGpio->SetCtlMode(PinN, 0b0100UL); // Floating input
    else {
        PGpio->SetCtlMode(PinN, 0b1000); // Input with pullup/pulldown
        if(PullUpDown == PullDown) PGpio->OCTL &= ~(1UL << PinN);
        else PGpio->OCTL |= 1UL << PinN;
    }
}

void SetupAnalog(GPIO_TypeDef *PGpio, const uint32_t PinN) {
    RCU->EnGpio(PGpio);
    PGpio->SetCtlMode(PinN, 0); // Clear both mode and cnf
}

void SetupAlterFunc(GPIO_TypeDef *PGpio, const uint32_t PinN, const OutMode_t OutMode, const Speed_t ASpeed) {
    RCU->EnGpio(PGpio);
    RCU->EnAFIO();
    uint32_t CtlMode = ((uint32_t)OutMode << 2) | 0b1000UL | ((uint32_t)ASpeed & 0b11UL);
    PGpio->SetCtlMode(PinN, CtlMode);
    if(ASpeed == speed120MHz) PGpio->SPD |= 1UL << PinN;
    else PGpio->SPD &= ~(1UL << PinN);
}

} // namespace Gpio

#if 1 // =========================== I2C =======================================
#if I2C0_ENABLED
i2c_t i2c0 {I2C0, I2C0_SCL, I2C0_SDA};
#endif
#if I2C1_ENABLED
i2c_t i2c1 {I2C1, I2C1_SCL, I2C1_SDA};
#endif

enum i2cState_t {istIdle, istWriteRead, istWriteWrite, istRead, istWrite, istFailure};

void i2c_t::Init() {
    Standby();
    Resume();
#if I2C_USE_SEMAPHORE
    chBSemObjectInit(&BSemaphore, NOT_TAKEN);
#endif
}

void i2c_t::Standby() {
    if(pi2c == I2C0) {
        RCU->ResI2C0();
        RCU->DisI2C0();
        Nvic::DisableVector(I2C0_EV_IRQn);
        Nvic::DisableVector(I2C0_ER_IRQn);
    }
    else if(pi2c == I2C1) {
        RCU->ResI2C1();
        RCU->DisI2C1();
        Nvic::DisableVector(I2C1_EV_IRQn);
        Nvic::DisableVector(I2C1_ER_IRQn);
    }
    // Disable GPIOs
    Gpio::SetupAnalog(PGpioScl, PinScl);
    Gpio::SetupAnalog(PGpioSda, PinSda);
}

void i2c_t::Resume() {
    // GPIO
    Gpio::SetupAlterFunc(PGpioScl, PinScl, Gpio::OpenDrain, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(PGpioSda, PinSda, Gpio::OpenDrain, Gpio::speed50MHz);
#if I2C_USE_SEMAPHORE
    chBSemObjectInit(&BSemaphore, NOT_TAKEN);
#endif
    // ==== Clock and reset ====
    if(pi2c == I2C0) {
        RCU->EnI2C0();
        RCU->ResI2C0();
        Nvic::EnableVector(I2C0_EV_IRQn, IRQ_PRIO_MEDIUM);
        Nvic::EnableVector(I2C0_ER_IRQn, IRQ_PRIO_MEDIUM);
    }
    else if(pi2c == I2C1) {
        RCU->EnI2C1();
        RCU->ResI2C1();
        Nvic::EnableVector(I2C1_EV_IRQn, IRQ_PRIO_MEDIUM);
        Nvic::EnableVector(I2C1_ER_IRQn, IRQ_PRIO_MEDIUM);
    }
    pi2c->CTL0 = 0;  // Clear I2CEN bit => disable
    // Minimum periph clock is 2 MHz
    uint32_t ClkMhz = Clk::APB1FreqHz / 1000000;
    if     (ClkMhz < I2CCLK_MIN_MHz) ClkMhz = I2CCLK_MIN_MHz;
    else if(ClkMhz > I2CCLK_MAX_MHz) ClkMhz = I2CCLK_MAX_MHz;
    SET_BITS(pi2c->CTL1, 0b111111UL, ClkMhz, 0);
    // Setup depending on speed
    uint32_t clkc;
#if I2C_BAUDRATE_HZ <= 100000UL
    // RiseTime
    uint32_t risetime = (uint32_t)((Clk::APB1FreqHz / 1000000U) + 1UL); // the maximum SCL rise time is 1000ns in standard mode
    if(risetime <= I2CCLK_MIN_MHz) risetime = I2CCLK_MIN_MHz;
    else if(risetime >= I2CCLK_MAX_MHz) risetime = I2CCLK_MAX_MHz;
    pi2c->RT = risetime;
    // ClkC
    clkc = (uint32_t)(Clk::APB1FreqHz / (I2C_BAUDRATE_HZ * 2UL));
    if(clkc < 0x04UL) clkc = 0x04UL;
    pi2c->SetClkc(clkc);
#else // 400k or higher
#if I2C_DUTYCYC == I2C_DUTYCYCLE_2
    clkc = (uint32_t)(Clk::APB1FreqHz / (I2C_BAUDRATE_HZ * 3UL));
    pi2c->SetDutycycle2();
#else // 16/9
    clkc = (uint32_t)(Clk::APB1FreqHz / (I2C_BAUDRATE_HZ * 25UL));
    pi2c->SetDutycycle16d9();
#endif
#if I2C_BAUDRATE_HZ <= 400000UL
    // RiseTime: the maximum SCL rise time is 300ns in fast mode
    pi2c->RT = (uint32_t)(((ClkMhz * 300UL) / 1000UL) + 1UL);
#else // Fast+
    // RiseTime: the maximum SCL rise time is 120ns
    pi2c->RT = (uint32_t)((ClkMhz * 120UL) / 1000UL + 1UL);
    pi2c->EnFastPlus();
#endif
    if(clkc == 0UL) clkc = 1UL;
    pi2c->SetSpeedFast();
    pi2c->SetClkc(clkc);
#endif
}

void i2c_t::ScanBus(Shell_t *PShell) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return;
#endif
    PShell->Print("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    uint32_t AddrHi, Addr;
    pi2c->Enable();
    for(AddrHi = 0; AddrHi < 0x80; AddrHi += 0x10) {
        PShell->Print("\r%02X: ", AddrHi);
        for(uint32_t n=0; n < 0x10; n++) {
            Addr = AddrHi + n;
            if(Addr <= 0x01 or Addr > 0x77) Printf("   ");
            else {
                // Try to get response from addr
                if(IBusyWait() != retv::Ok) {
                    PShell->Print("i2cBusyWait\r");
                    return;
                }
                pi2c->ClearStatFlags();
                pi2c->FlushDatabuf();
                pi2c->ClearAddrFlag();
                // Start transmission
                pi2c->SendStart();
                if(IWaitStartSent() != retv::Ok) continue;
                pi2c->SendAddrWithWrite(Addr);
                if(IWaitAddrSent() == retv::Ok) Printf("%02X ", Addr);
                else Printf("__ ");
                pi2c->SendStop();
            }
        } // for lo
    } // for hi
    PShell->PrintEOL();
    pi2c->Disable();
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
}

// ==== Flag operations ====
#define RETRY_CNT       450

retv i2c_t::IBusyWait(uint32_t Timeout_ms) {
    for(uint32_t i=0; i<Timeout_ms; i++) {
        if(!pi2c->IsBusy()) return retv::Ok;
        Sys::SleepMilliseconds(1);
    }
    return retv::Timeout;
}

retv i2c_t::IWaitStartSent() {
    uint32_t RetryCnt = RETRY_CNT;
    while(RetryCnt--)
        if(pi2c->IsStartSentAndBusyMaster())
            return retv::Ok;
    return retv::Fail;
}

retv i2c_t::IWaitAddrSent() {
    uint32_t RetryCnt = RETRY_CNT;
    while(RetryCnt-- and !pi2c->IsNACK())
        if(pi2c->IsAddrSentAndACKed()) return retv::Ok;
    return retv::Fail;
}

retv i2c_t::WriteRead(uint8_t Addr, uint8_t *WPtr,  uint32_t WLength, uint8_t *RPtr, uint32_t RLength, uint32_t Timeout_ms) {
#if I2C_USE_SEMAPHORE
    if(chBSemWait(&BSemaphore) != MSG_OK) return retvBusy;
#endif
    retv Rslt = retv::Ok;
    if(IBusyWait() == retv::Ok) {
        // Clear flags and buffer
        pi2c->ClearStatFlags();
        pi2c->FlushDatabuf();
        pi2c->ClearAddrFlag();
        // Prepare context
        IAddr = Addr << 1; // Addr shifted & WriteBit (LSB = 0)
        IBufW.Ptr = WPtr;
        IBufW.Sz = WLength;
        IBufR.Ptr = RPtr;
        IBufR.Sz = RLength;
        // Start transmission
        Sys::Lock();
        PThd = Sys::GetSelfThd();
        pi2c->EnAllIRQs();
        pi2c->Enable();
        pi2c->SendStart();
        Rslt = Sys::SleepS(TIME_MS2I(Timeout_ms));
        // Will be here after wake from IRQ
        if(Rslt == retv::Timeout) {
            DisableAndClearIRQs();
            pi2c->Disable();
        }
        Sys::Unlock();
    }
    else Rslt = retv::Busy;
#if I2C_USE_SEMAPHORE
    chBSemSignal(&BSemaphore);
#endif
    return Rslt;
}

retv i2c_t::Write(uint8_t Addr, uint8_t *WPtr,  uint32_t WLength, uint32_t Timeout_ms) {
    return WriteRead(Addr, WPtr, WLength, nullptr, 0, Timeout_ms);
}

retv i2c_t::Read(uint8_t Addr, uint8_t *RPtr, uint32_t RLength, uint32_t Timeout_ms) {
    return WriteRead(Addr, nullptr, 0, RPtr, RLength, Timeout_ms);
}

retv i2c_t::WriteBytes(uint8_t Addr, uint32_t ByteCnt, ...) {
    uint8_t FBuf[ByteCnt], *p = FBuf;
    va_list args;
    va_start(args, ByteCnt);
    while(p < &FBuf[ByteCnt]) *p++ = va_arg(args, uint32_t);
    va_end(args);
    return Write(Addr, FBuf, ByteCnt);
}

// ==== IRQs ====
void i2c_t::DisableAndClearIRQs() {
    pi2c->DisAllIRQs();
    if(pi2c == I2C0) {
        Nvic::ClearPending(I2C0_EV_IRQn);
        Nvic::ClearPending(I2C0_ER_IRQn);
    }
    else if(pi2c == I2C1) {
        Nvic::ClearPending(I2C1_EV_IRQn);
        Nvic::ClearPending(I2C1_ER_IRQn);
    }
}

void i2c_t::OnTransmissionEnd(retv Rslt) {
    pi2c->SendStop();
    DisableAndClearIRQs();
    Sys::LockFromIRQ();
    Sys::WakeI(&PThd, Rslt);
    Sys::UnlockFromIRQ();
}

void i2c_t::IProcessIRQ() {
    Sys::IrqPrologue();
    volatile uint32_t stat0 = pi2c->STAT0;
    // Check errors
    if(stat0 & I2C_STAT0_AERR) OnTransmissionEnd(retv::NACK);
    else if(stat0 & (I2C_STAT0_BERR | I2C_STAT0_LOSTARB | I2C_STAT0_OUERR | I2C_STAT0_PECERR)) OnTransmissionEnd(retv::Fail);
    // No errors, process event
    else {
        // Start sent, send addr
        if(stat0 & I2C_STAT0_SBSEND) {
            if(IBufW.Sz == 0 and IBufR.Sz > 0) IAddr |= 0x01; // Set ReadBit (LSB = 1)
            (void)pi2c->STAT0; // To clear SBSEND flag, read STAT0 and write DATA __immediately__
            pi2c->SendData(IAddr);
        }
        // Address sent
        else if(stat0 & I2C_STAT0_ADDSEND) {
            (void)pi2c->STAT0; // }
            (void)pi2c->STAT1; // } Clear ADDSEND flag
            if(IBufW.Sz == 0) { // Nothing to write. But maybe there is something to read.
                if(IBufR.Sz == 0) OnTransmissionEnd(retv::Ok); // No data, nothing to do
                else { // Need to read
                    pi2c->EnBufferIRQ(); // Enable data receiving IRQ
                    if(IBufR.Sz == 1) {  // If it is a single byte to be received,
                        pi2c->DisAck();  // ...do not ACK it.
                        pi2c->SendStop(); // And send STOP after transmission. See Prog Model in datasheet.
                    }
                    else pi2c->EnAck(); // If many bytes are to be received, ACK them
                }
            }
        }
        // TX buf empty, send bytes until flag is set
        if(stat0 & I2C_STAT0_TBE) {
            while(IBufW.Sz > 0 and (pi2c->STAT0 & I2C_STAT0_TBE)) { // Fill i2c buf until flag allows it
                pi2c->SendData(*IBufW.Ptr);
                IBufW.Ptr++;
                IBufW.Sz--;
            }
            if(IBufW.Sz == 0) { // Tx buf ended
                pi2c->DisBufferIRQ(); // Do not bother MCU with BufEmpty IRQs
                if(IBufR.Sz > 0) pi2c->SendStart(); // Send repeated start to start rx transaction
            }
        }
        // RX buf contains data
        if(stat0 & I2C_STAT0_RBNE) {
            *IBufR.Ptr = pi2c->GetData();
            IBufR.Ptr++;
            IBufR.Sz--;
            if(IBufR.Sz == 0) OnTransmissionEnd(retv::Ok);
            else if(IBufR.Sz == 1) {
                pi2c->DisAck();   // Do not ack last byte
                pi2c->SendStop(); // ...and send STOP after transmission
            }
        }
        // Byte is sent or received
        if((stat0 & I2C_STAT0_BTC) and IBufW.Sz == 0 and IBufR.Sz == 0) OnTransmissionEnd(retv::Ok);
    } // no errors
    Sys::IrqEpilogue();
}

extern "C" {
#if I2C0_ENABLED
void I2C0_EV_IRQHandler() { i2c0.IProcessIRQ(); }
void I2C0_ER_IRQHandler() { i2c0.IProcessIRQ(); }
#endif

#if I2C1_ENABLED
void I2C1_EV_IRQHandler() { i2c1.IProcessIRQ(); }
void I2C1_ER_IRQHandler() { i2c1.IProcessIRQ(); }
#endif
} // extern C
#endif

#if 1 // =========================== DMA =======================================
static const IRQn_Type DmaIrqNum[DMA_CHNL_CNT] = {
        DMA0_Channel0_IRQn, DMA0_Channel1_IRQn, DMA0_Channel2_IRQn, DMA0_Channel3_IRQn,
        DMA0_Channel4_IRQn, DMA0_Channel5_IRQn, DMA0_Channel6_IRQn,
        DMA1_Channel0_IRQn, DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn,
};

struct DmaIrqHandler_t {
    ftVoidPVoidW32 Handler = nullptr;
    void *Param = nullptr;
    uint32_t Prio = IRQ_PRIO_LOW;
};

static DmaIrqHandler_t DmaIrqHandler[DMA_CHNL_CNT];

DMA_t::DMA_t(DMAChannel_t *APChnl, ftVoidPVoidW32 PIrqFunc, void *PIrqParam, uint32_t AIrqPrio) {
    PChnl = APChnl;
    // Calculate ChnlN
    if((uint32_t)APChnl <= (uint32_t)DMA0_Channel6_BASE)
        ChnlN = ((uint32_t)APChnl - (uint32_t)DMA0_Channel0_BASE) / 0x14UL; // 0x14 is distance between channels, see datasheet
    else ChnlN = ((uint32_t)APChnl - (uint32_t)DMA1_Channel0_BASE) / 0x14UL;
    // Setup IRQ
    DmaIrqHandler[ChnlN].Handler = PIrqFunc;
    DmaIrqHandler[ChnlN].Param = PIrqParam;
    DmaIrqHandler[ChnlN].Prio = AIrqPrio;
}

void DMA_t::Init() const {
    RCU->EnDMAs(); // En DMA1 & DMA2 clocking
    if(DmaIrqHandler[ChnlN].Handler != nullptr) {
        Nvic::EnableVector(DmaIrqNum[ChnlN], DmaIrqHandler[ChnlN].Prio);
    } // if irq
    PChnl->CTL = 0; // Reset value
}

void DMA_t::Init(volatile void* PeriphAddr, void* MemAddr, uint32_t AMode, uint16_t Cnt) const {
    Init();
    SetPeriphAddr(PeriphAddr);
    SetMemoryAddr(MemAddr);
    SetMode(AMode);
    SetTransferDataCnt(Cnt);
}

void DMA_t::ClearIrq() const { // DMA0: chnls [0;6]; DMA1: chnls [7;11]
    if(ChnlN <= 6) DMA0->INTC = 0b1111UL << ChnlN;  // [0;6]
    else DMA1->INTC = 0b1111UL << (ChnlN - 7);      // [7;11]
}

void DMA_t::DisableAndClearIRQ() const {
    PChnl->CTL &= ~0b1111UL; // TEIE | HTIE | TCIE | EN
    ClearIrq();
}

// ==== IRQs ====
#define DMA_IRQ_HANDLER(DmaN, ChnlN) \
    void DMA##DmaN##_Channel##ChnlN##_IRQHandler() { \
        uint32_t flags = (DMA##DmaN->INTF >> (ChnlN * 4)) & 0b1111UL; \
        DMA##DmaN->INTC = 1UL << (ChnlN * 4); /* Clear all irq flags */ \
        ftVoidPVoidW32 func = DmaIrqHandler[DMA_CHNL(DmaN, ChnlN)].Handler; \
        if(func) func(DmaIrqHandler[DMA_CHNL(DmaN, ChnlN)].Param, flags); \
    }

extern "C" {
#if DMA0_CH0_IRQ_EN
DMA_IRQ_HANDLER(0, 0);
#endif
#if DMA0_CH1_IRQ_EN
DMA_IRQ_HANDLER(0, 1);
#endif
#if DMA0_CH2_IRQ_EN
DMA_IRQ_HANDLER(0, 2);
#endif
#if DMA0_CH3_IRQ_EN
DMA_IRQ_HANDLER(0, 3);
#endif
#if DMA0_CH4_IRQ_EN
DMA_IRQ_HANDLER(0, 4);
#endif
#if DMA0_CH5_IRQ_EN
DMA_IRQ_HANDLER(0, 5);
#endif
#if DMA0_CH6_IRQ_EN
DMA_IRQ_HANDLER(0, 6);
#endif

#if DMA1_CH0_IRQ_EN
DMA_IRQ_HANDLER(1, 0);
#endif
#if DMA1_CH1_IRQ_EN
DMA_IRQ_HANDLER(1, 1);
#endif
#if DMA1_CH2_IRQ_EN
DMA_IRQ_HANDLER(1, 2);
#endif
#if DMA1_CH3_IRQ_EN
DMA_IRQ_HANDLER(1, 3);
#endif
#if DMA1_CH4_IRQ_EN
DMA_IRQ_HANDLER(1, 4);
#endif
} // extern C
#endif // DMA

#if 1 // ================================= SPI =================================
void Spi_t::Setup(BitOrder BitOrdr, cpol CPOL, cpha CPHA, int32_t Bitrate_Hz, BitNumber BitNum) const {
    RCU->EnSpi(PSpi); // Clocking
    // Mode: Master, NSS software controlled and is 1, 8bit, NoCRC, FullDuplex
    PSpi->CTL0 = SPI_CTL0_SWNSSEN | SPI_CTL0_SWNSS | SPI_CTL0_MSTMOD;
    PSpi->CTL1 = 0; // All irqs and DMA are disabled
    if(BitOrdr == BitOrder::LSB) PSpi->CTL0 |= SPI_CTL0_LF;    // MSB/LSB
    if(CPOL == cpol::IdleHigh)   PSpi->CTL0 |= SPI_CTL0_CKPL;  // CPOL
    if(CPHA == cpha::SecondEdge) PSpi->CTL0 |= SPI_CTL0_CKPH;  // CPHA
    // Baudrate
    int32_t div;
    if(PSpi == SPI0) div = Clk::APB2FreqHz / Bitrate_Hz;
    else div = Clk::APB1FreqHz / Bitrate_Hz;
    if     (div > 128) PSpi->CTL0 |= 0b111UL << 3; // PCLK/256
    else if(div > 64)  PSpi->CTL0 |= 0b110UL << 3; // PCLK/128
    else if(div > 32)  PSpi->CTL0 |= 0b101UL << 3; // PCLK/64
    else if(div > 16)  PSpi->CTL0 |= 0b100UL << 3; // PCLK/32
    else if(div > 8)   PSpi->CTL0 |= 0b011UL << 3; // PCLK/16
    else if(div > 4)   PSpi->CTL0 |= 0b010UL << 3; // PCLK/8
    else if(div > 2)   PSpi->CTL0 |= 0b001UL << 3; // PCLK/4
    // Otherwise, div2 is used, which is 0b000.
    // Bit number
    if(BitNum == BitNumber::n16) PSpi->CTL0 |= SPI_CTL0_FF16;
}

void Spi_t::EnQuadWrite() {
    PSpi->WaitForTBEHiAndTransLo();
    PSpi->QCTL = SPI_QCTL_QMOD;
}

void Spi_t::EnQuadRead() {
    PSpi->WaitForTransLo();
    PSpi->QCTL = SPI_QCTL_QMOD | SPI_QCTL_QRD;
}
void Spi_t::DisQuad() {
    PSpi->WaitForTBEHiAndTransLo();
    PSpi->QCTL = 0;
}
#endif

namespace Clk { // ======================== Clocking ===========================
uint32_t AHBFreqHz, APB1FreqHz, APB2FreqHz;

void UpdateFreqValues() {
    uint32_t CkSysHz = RCU->GetCkSys();
    // Calculate AHB freq
    const uint8_t AHB_div[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    uint32_t Indx = GET_BITS(RCU->CFG0, 0b1111UL, 4);
    AHBFreqHz = CkSysHz >> AHB_div[Indx];
    // Calculate APB freq
    const uint8_t APB_div[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    Indx = GET_BITS(RCU->CFG0, 0b111UL, 8);
    APB1FreqHz = AHBFreqHz >> APB_div[Indx];
    Indx = GET_BITS(RCU->CFG0, 0b111UL, 11);
    APB2FreqHz = AHBFreqHz >> APB_div[Indx];
}

void PrintFreqs() {
    Printf("AHBFreq=%uMHz; APB1Freq=%uMHz; APB2Freq=%uMHz\r",
            AHBFreqHz/1000000, APB1FreqHz/1000000, APB2FreqHz/1000000);
}

uint32_t GetTimInputFreq(const uint32_t TimerN) {
    if(TimerN == 0 or (TimerN >= 7 and TimerN <= 10)) {
        uint32_t Indx = GET_BITS(RCU->CFG0, 0b111UL, 11); // APB2 divider
        return (Indx < 0b100UL)? APB2FreqHz : APB2FreqHz * 2; // x1 if no divider, x2 if divided
    }
    else {
        uint32_t Indx = GET_BITS(RCU->CFG0, 0b111UL, 8); // APB1 divider
        return (Indx < 0b100UL)? APB1FreqHz : APB1FreqHz * 2; // x1 if no divider, x2 if divided
    }
}

uint32_t GetTimInputFreq(const TIM_TypeDef *PTimer) {
    if(PTimer == TIM0 or PTimer == TIM7 or PTimer == TIM8 or PTimer == TIM9 or PTimer == TIM10) {
        uint32_t Indx = GET_BITS(RCU->CFG0, 0b111UL, 11); // APB2 divider
        return (Indx < 0b100UL)? APB2FreqHz : APB2FreqHz * 2; // x1 if no divider, x2 if divided
    }
    else {
        uint32_t Indx = GET_BITS(RCU->CFG0, 0b111UL, 8); // APB1 divider
        return (Indx < 0b100UL)? APB1FreqHz : APB1FreqHz * 2; // x1 if no divider, x2 if divided
    }
}

} // namespace

namespace Nvic { // ======================== NVIC ==============================

void EnableVector(IRQn_Type IrqN, uint32_t prio) {
    uint32_t n = (uint32_t)IrqN;
#if defined(__CORE_CM0_H_GENERIC)
    NVIC->IP[_IP_IDX(n)] = (NVIC->IP[_IP_IDX(n)] & ~(0xFFU << _BIT_SHIFT(n))) |
                         (NVIC_PRIORITY_MASK(prio) << _BIT_SHIFT(n));
#else
    NVIC->IP[n] = NVIC_PRIORITY_MASK(prio);
#endif
    NVIC->ICPR[n >> 5U] = 1U << (n & 0x1FU);
    NVIC->ISER[n >> 5U] = 1U << (n & 0x1FU);
}

void DisableVector(IRQn_Type IrqN) {
    uint32_t n = (uint32_t)IrqN;
    NVIC->ICER[n >> 5U] = 1U << (n & 0x1FU);
#if defined(__CORE_CM0_H_GENERIC)
    NVIC->IP[_IP_IDX(n)] = NVIC->IP[_IP_IDX(n)] & ~(0xFFU << _BIT_SHIFT(n));
  #else
    NVIC->IP[n] = 0U;
#endif
}

void SetSystemHandlerPriority(uint32_t handler, uint32_t prio) {
#if defined(__CORE_CM0_H_GENERIC)
    SCB->SHP[_SHP_IDX(handler)] = (SCB->SHP[_SHP_IDX(handler)] & ~(0xFFU << _BIT_SHIFT(handler))) |
                                (NVIC_PRIORITY_MASK(prio) << _BIT_SHIFT(handler));
#elif defined(__CORE_CM7_H_GENERIC)
    SCB->SHPR[handler] = NVIC_PRIORITY_MASK(prio);
#else
    SCB->SHP[handler] = NVIC_PRIORITY_MASK(prio);
#endif
}

void ClearPending(IRQn_Type IrqN) { NVIC->ICPR[(uint32_t)IrqN >> 5] = 1 << ((uint32_t)IrqN & 0x1F); }

} // namespace

// PWM
void PinOutputPWM_t::Init() const {
    // GPIO
    Gpio::SetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType);
    // Timer
    RCU->EnTimer(ISetup.PTimer);
    ISetup.PTimer->CCHP = 0xC000;
    ISetup.PTimer->CTL0 |= 1UL << 7; // Auto-reload shadow enable
    ISetup.PTimer->SetTopValue(ISetup.TopValue);
    uint32_t tmp = (ISetup.Inverted == invInverted)? 0b111UL : 0b110UL; // PWM mode 1 or 2
    switch(ISetup.TimerChnl) {
        case 0:
            ISetup.PTimer->CHCTL0 |= (tmp << 4);
            ISetup.PTimer->CHCTL2 |= 1UL << 0; // Set CH0EN
            break;
        case 1:
            ISetup.PTimer->CHCTL0 |= (tmp << 12);
            ISetup.PTimer->CHCTL2 |= 1UL << 4; // Set CH1EN
            break;
        case 2:
            ISetup.PTimer->CHCTL1 |= (tmp << 4);
            ISetup.PTimer->CHCTL2 |= 1UL << 8; // Set CH2EN
            break;
        case 3:
            ISetup.PTimer->CHCTL1 |= (tmp << 12);
            ISetup.PTimer->CHCTL2 |= 1UL << 12; // Set CH0EN
            break;
        default: break;
    }
    ISetup.PTimer->Enable();
}
