/*
 * ir.cpp
 *
 *  Created on: 04.07.2013
 *      Author: kreyl
 */

#include "ir.h"
#include "gd_uart.h"

#if IR_TX_ENABLED // ========================== IR TX ==========================
namespace irLed {

#define IRLED_DMA_MODE  DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_TCIE

HwTim SamplingTmr{TMR_DAC_SMPL};
ftVoidVoid ICallbackI = nullptr;
static uint32_t TransactionSz;
static void DmaTxEndIrqHandler(void *p, uint32_t flags);
static const DMA_t DmaTx {DAC_DMA, DmaTxEndIrqHandler, nullptr, IRQ_PRIO_MEDIUM};
// Samples count
static struct {
    int32_t Header, Space, One, Zero, Pause;
} NSamples;

// DAC buf: reserve size fo 56kHz (969)
#define DAC_BUF_SZ          999UL

union DacSamplePair_t {
    uint32_t W32;
    struct {
        uint8_t OnPwr1;
        uint8_t OffPwr1;
        uint8_t OnPwr2;
        uint8_t OffPwr2;
    } __attribute__((packed));
    DacSamplePair_t& operator = (const DacSamplePair_t& Right) {
        W32 = Right.W32;
        return *this;
    }
    DacSamplePair_t() : W32(0) {}
    DacSamplePair_t(uint8_t Pwr) : OnPwr1(Pwr), OffPwr1(0), OnPwr2(Pwr), OffPwr2(0) {}
} __attribute__((packed));

DacSamplePair_t DacBuf[DAC_BUF_SZ];

static void DmaTxEndIrqHandler(void *p, uint32_t flags) {
    Sys::LockFromIRQ();
    SamplingTmr.Disable();
    DmaTx.Disable();
    if(ICallbackI) ICallbackI();
    Sys::UnlockFromIRQ();
}

static inline void StartTx() {
    DmaTx.SetMemoryAddr(DacBuf);
    DmaTx.SetTransferDataCnt(TransactionSz);
    DmaTx.Enable();
    SamplingTmr.Enable();
}

void Init() {
    // ==== GPIO ====
    // Once the DAC channel is enabled, the corresponding GPIO pin is automatically
    // connected to the DAC converter. In order to avoid parasitic consumption,
    // the GPIO pin should be configured in analog.
    Gpio::SetupAnalog(IR_LED);
    // ==== DAC ====
    RCU->EnDAC();
    DAC->SetTrigger0(DAC_TypeDef::Trigger::Tim6TRGO);
    DAC->EnTrigger0();
    DAC->EnDma0();
    DAC->Enable0();
    // ==== DMA ====
    DmaTx.Init(&DAC->DAC0_R8DH, IRLED_DMA_MODE);
    // ==== Sampling timer ====
    SamplingTmr.Init();
    SamplingTmr.SelectMasterMode(HwTim::MasterMode::Update);
}

void SetCarrierFreq(uint32_t CarrierFreqHz) {
    uint32_t SamplingFreqHz = CarrierFreqHz * 2;
    SamplingTmr.SetUpdateFreqChangingTopValue(SamplingFreqHz);
    // Every SamplePair contains 4 actual samples
    NSamples.Header = (((IR_HEADER_uS * CarrierFreqHz) + 1999999L) / 2000000L);
    NSamples.Space  = (((IR_SPACE_uS  * CarrierFreqHz) + 1999999L) / 2000000L);
    NSamples.Zero   = (((IR_ZERO_uS   * CarrierFreqHz) + 1999999L) / 2000000L);
    NSamples.One    = (((IR_ONE_uS    * CarrierFreqHz) + 1999999L) / 2000000L);
    NSamples.Pause  = (((IR_PAUSE_AFTER_uS * CarrierFreqHz) + 1999999UL) / 2000000UL);
    // Check if buf sz is enough
    uint32_t N = NSamples.Header + NSamples.Space + (NSamples.One + NSamples.Space) * IR_BIT_CNT_MAX + NSamples.Pause;
    if(N > DAC_BUF_SZ) Printf("IR TX DAC Buf Sz too small: %d < %d\r\n", DAC_BUF_SZ, N);
}

// Power is 8-bit DAC value
void TransmitWord(uint16_t wData, int32_t BitCnt, uint8_t Power, ftVoidVoid CallbackI) {
    ICallbackI = CallbackI;
    // ==== Fill buffer depending on data ====
    DacSamplePair_t *p = DacBuf, ISampleCarrier{Power}, ISampleSpace{0};
    int32_t i, j;
    // Put header
    for(i=0; i<NSamples.Header; i++) *p++ = ISampleCarrier;
    for(i=0; i<NSamples.Space; i++)  *p++ = ISampleSpace;
    // Put data
    for(j=0; j<BitCnt; j++) {
        // Carrier
        if(wData & 0x8000) { for(i=0; i<NSamples.One;  i++) *p++ = ISampleCarrier; }
        else               { for(i=0; i<NSamples.Zero; i++) *p++ = ISampleCarrier; }
        // Space
        for(i=0; i<NSamples.Space; i++)  *p++ = ISampleSpace;
        wData <<= 1;
    }
    // Put pause
    for(i=0; i<NSamples.Pause; i++) *p++ = ISampleSpace;
    // ==== Start transmission ====
    TransactionSz = (p - DacBuf) * 4; // Every sample pair contains 4 actual samples
    StartTx();
}

void ResetI() {
    DmaTx.Disable();
    SamplingTmr.Disable();
    DAC->PutDataR0(0);
    SamplingTmr.GenerateUpdateEvt();
}
} // namespace
#endif

#if IR_RX_ENABLED // ========================== IR RX ==========================
/* ==== Timer ====
Here Input1 is used for resetting on falling edge and capturing on rising one.
Two outputs of EdgeDetector1 are used: CI1FE0 and CI1FE1.
Edge polarity is set in CHCTL2 reg: for CI1FE0 using CH0P bit, and for
CI1FE1 using CH1P bit. (CH0P sets edge polarity for both CI0FE0 and CI1FE0 signals;
CH1P - for CI0FE1 and CI1FE1; and so on).
______        ________
      |______|
      ^      ^
   TI2FP2   TI2FP1
   CI1FE1   CI1FE0
   Trigger  Capture
   Reset    CCR1 => DMA req CCR1 => TIMx Ch1 (not Ch2!) request
*/
namespace irRcvr {

ftVoidU32 ICallbackI;
HwTim TmrRx{TMR_IR_RX};

void Init(ftVoidU32 CallbackI) {
    ICallbackI = CallbackI;
    Gpio::SetupInput(IR_RX_DATA_PIN, Gpio::PullUp);
    TmrRx.Init();
    TmrRx.SetTopValue(0xFFFF); // Maximum
    TmrRx.SetInputFreqChangingPrescaler(1000000);  // Input Freq: 1 MHz => one tick = 1 uS
    // Setup input mode for Channel0: capture Input1 (not Input0) on rising edge
    TmrRx.SetChnlMode(0, HwTim::ChnlMode::CI1FE0); // Chnl0 is input, capture on Input1's CI1FE0 signal
    TmrRx.SetInputActiveEdge(0, RiseFall::Rising); // CI1FE0 is Active Rising (CI0FE0 is the same, but it is not used)
    // Reset timer on trigger; trigger is falling edge on CI1FE1
    TmrRx.SetTriggerInput(HwTim::TriggerIn::CI1FE1); // Use Input1's CI1FE1 as TriggerIn
    TmrRx.SetInputActiveEdge(1, RiseFall::Falling);
    // Configure slave mode controller in Restart mode
    TmrRx.SelectSlaveMode(HwTim::SlaveMode::Restart);
    // Enable the capture on channel 0
    TmrRx.EnChnl(0);
    // IRQ
    TmrRx.EnableIrqOnCompare0();
    Nvic::EnableVector(TMR_IR_RX_IRQ, IRQ_PRIO_HIGH);
    // Start capture
    TmrRx.Enable();
}

void SetCallback(ftVoidU32 CallbackI) { ICallbackI = CallbackI; }

// Parsing
static int32_t IBitCnt = -1, StopRemainder = 0; // Header not received
static uint32_t IRxData;
static systime_t RxStartTime = 0;

static inline void ProcessDurationI(uint32_t Dur) {
//    PrintfI("%d\r", Dur);
    if(IS_LIKE(Dur, IR_HEADER_uS, IR_DEVIATION_uS)) { // Header rcvd
        IBitCnt = 16;
        StopRemainder = 0;
        IRxData = 0;
        RxStartTime = Sys::GetSysTimeX();
    }
    // Ignore received if error occured previously
    else if(IBitCnt != -1) {
        if(Sys::TimeElapsedSince(RxStartTime) < TIME_MS2I(IR_RX_PKT_TIMEOUT_MS)) {
            uint32_t bit;
            if     (IS_LIKE(Dur, IR_ZERO_uS, IR_DEVIATION_uS)) bit = 0UL;
            else if(IS_LIKE(Dur, IR_ONE_uS,  IR_DEVIATION_uS)) bit = 1UL;
            else { IBitCnt = -1; return; } // Bad duration
            // Find out expected bit cnt
            if(IBitCnt == 16 and bit == 0) StopRemainder = 2; // if first bit is 0, 14 bits are expected
            IBitCnt--;
            IRxData |= bit << IBitCnt;
            if(IBitCnt <= StopRemainder) { // Reception completed
                if(ICallbackI) ICallbackI(IRxData);
                IBitCnt = -1; // Wait header
            }
            else RxStartTime = Sys::GetSysTimeX(); // Restart timeout
        }
        else IBitCnt = -1; // timeout occured
    }
}

} // namespace

// ==== IRQ ====
extern "C"
void TMR_IR_RX_IRQ_HNDLR() {
    Sys::IrqPrologue();
    Sys::LockFromIRQ();
    irRcvr::TmrRx.ClearCompare0IrqPendingBit();
    irRcvr::ProcessDurationI(irRcvr::TmrRx.GetChnl0Value());
    Sys::UnlockFromIRQ();
    Sys::IrqEpilogue();
}
#endif
