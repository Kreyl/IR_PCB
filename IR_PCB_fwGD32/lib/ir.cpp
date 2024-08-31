/*
 * ir.cpp
 *
 *  Created on: 04.07.2013
 *      Author: kreyl
 */

#include "ir.h"
#include "gd_uart.h"
#include "Settings.h"

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
    int32_t header, space, one, zero, pause;
} samples_cnt;

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
    gpio::SetupAnalog(IR_LED);
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

void SetCarrierFreq(uint32_t carrier_freq_Hz) {
    uint32_t sampling_freq_Hz = carrier_freq_Hz * 2;
    SamplingTmr.SetUpdateFreqChangingTopValue(sampling_freq_Hz);
    // Every SamplePair contains 4 actual samples
    samples_cnt.header = (((kIr::Header_us * carrier_freq_Hz) + 1999999L) / 2000000L);
    samples_cnt.space  = (((kIr::Space_us  * carrier_freq_Hz) + 1999999L) / 2000000L);
    samples_cnt.zero   = (((kIr::Zero_us   * carrier_freq_Hz) + 1999999L) / 2000000L);
    samples_cnt.one    = (((kIr::One_us    * carrier_freq_Hz) + 1999999L) / 2000000L);
    samples_cnt.pause  = (((kIr::PauseAfter_us * carrier_freq_Hz) + 1999999UL) / 2000000UL);
    // Check if buf sz is enough
    uint32_t N = samples_cnt.header + samples_cnt.space + (samples_cnt.one + samples_cnt.space) * IR_BIT_CNT_MAX + samples_cnt.pause;
    if(N > DAC_BUF_SZ) Printf("IR TX DAC Buf Sz too small: %d < %d\r\n", DAC_BUF_SZ, N);
}

// Power is 8-bit DAC value
void TransmitWord(uint16_t wData, int32_t BitCnt, uint8_t Power, ftVoidVoid CallbackI) {
    ICallbackI = CallbackI;
    // ==== Fill buffer depending on data ====
    DacSamplePair_t *p = DacBuf, ISampleCarrier{Power}, ISampleSpace{0};
    int32_t i, j;
    // Put header
    for(i=0; i<samples_cnt.header; i++) *p++ = ISampleCarrier;
    for(i=0; i<samples_cnt.space; i++)  *p++ = ISampleSpace;
    // Put data
    for(j=0; j<BitCnt; j++) {
        // Carrier
        if(wData & 0x8000) { for(i=0; i<samples_cnt.one;  i++) *p++ = ISampleCarrier; }
        else               { for(i=0; i<samples_cnt.zero; i++) *p++ = ISampleCarrier; }
        // space
        for(i=0; i<samples_cnt.space; i++)  *p++ = ISampleSpace;
        wData <<= 1;
    }
    // Put pause
    for(i=0; i<samples_cnt.pause; i++) *p++ = ISampleSpace;
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
____        _____________________
    |______|   InterBitTimeot    ^
    ^      ^                     ^
 TI2FP2   TI2FP1                OVF => Timeout IRQ
 CI1FE1   CI1FE0
 Trigger  Capture
 Reset    CCR1 => Capture IRQ
*/
namespace irRcvr {

ftVoidU8U16 callbackI;
HwTim tmr_rx{TMR_IR_RX};

void Init() {
    gpio::SetupInput(IR_RX_DATA_PIN, gpio::PullUp);
    tmr_rx.Init();
    tmr_rx.DisableAutoreloadBuffering(); // To put there timeout immediately
    tmr_rx.SetTopValue(0xFFFF); // Maximum, just to start
    tmr_rx.SetUpdateSrcOvfOnly();
    tmr_rx.SetInputFreqChangingPrescaler(1000000);  // Input Freq: 1 MHz => one tick = 1 uS
    // Setup input mode for Channel0: capture Input1 (not Input0) on rising edge
    tmr_rx.SetChnlMode(0, HwTim::ChnlMode::CI1FE0); // Chnl0 is input, capture on Input1's CI1FE0 signal
    tmr_rx.SetInputActiveEdge(0, RiseFall::Rising); // CI1FE0 is Active Rising (CI0FE0 is the same, but it is not used)
    // Reset timer on trigger; trigger is falling edge on CI1FE1
    tmr_rx.SetTriggerInput(HwTim::TriggerIn::CI1FE1); // Use Input1's CI1FE1 as TriggerIn
    tmr_rx.SetInputActiveEdge(1, RiseFall::Falling);
    // Configure slave mode controller in Restart mode
    tmr_rx.SelectSlaveMode(HwTim::SlaveMode::Restart);
    // Enable the capture on channel 0
    tmr_rx.EnChnl(0);
    // IRQ
    tmr_rx.EnableIrqOnCompare0();
    Nvic::EnableVector(TMR_IR_RX_IRQ, IRQ_PRIO_HIGH);
    // Start capture
    tmr_rx.Enable();
}

// Parsing
static int32_t bit_indx = -1; // header not received
static uint32_t rx_data;

static void OnReceptionDone() {
//    PrintfI("ibto\r");
    int32_t bit_cnt = 16L - bit_indx;
    if(bit_cnt > 0 and bit_cnt <= 16) { // Some bits were received
        if(callbackI) callbackI(bit_cnt, rx_data);
    }
    tmr_rx.DisableIrqOnUpdate(); // Disable interbit timeout
    bit_indx = -1; // Reset reception
}

static void ProcessIrqI(uint32_t flags, uint32_t dur) {
//    PrintfI("0x%X %u\r",  flags, dur);
    if(IsLike<uint32_t>(dur, kIr::Header_us, *settings.ir_rx_deviation)) { // header rcvd
        bit_indx = 16;
        rx_data = 0;
        // Setup timeout
        tmr_rx.SetTopValue(dur + kIr::InterBitTimeot_us);
        tmr_rx.EnableIrqOnUpdate();
    }
    else { // Not header
        // Check if timeout occured, i.e. update IRQ fired
        if(HwTim::IsUpdateFlagSet(flags)) OnReceptionDone();
        else if(bit_indx != -1) { // No timeout; ignore received if not after header, or if bad length is received previously
            uint32_t bit;
            if     (IsLike<uint32_t>(dur, kIr::Zero_us, *settings.ir_rx_deviation)) bit = 0UL;
            else if(IsLike<uint32_t>(dur, kIr::One_us,  *settings.ir_rx_deviation)) bit = 1UL;
            else { // Bad duration
                bit_indx = -1;
                tmr_rx.DisableIrqOnUpdate(); // Disable interbit timeout
                return;
            }
            // Find out expected bit cnt
            bit_indx--;
            rx_data |= bit << bit_indx;
            if(bit_indx == 0) OnReceptionDone(); // Reception completed, as maximim amount of bits received
        }
    }
}

} // namespace

// ==== IRQ ====
extern "C"
void TMR_IR_RX_IRQ_HNDLR() {
    Sys::IrqPrologue();
    Sys::LockFromIRQ();
    uint32_t flags = irRcvr::tmr_rx.GetIrqFlags();
    irRcvr::tmr_rx.ClearAllIrqFlags();
    irRcvr::ProcessIrqI(flags, irRcvr::tmr_rx.GetChnl0Value());
    Sys::UnlockFromIRQ();
    Sys::IrqEpilogue();
}
#endif
