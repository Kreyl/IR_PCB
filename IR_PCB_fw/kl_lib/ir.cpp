/*
 * ir.cpp
 *
 *  Created on: 04.07.2013
 *      Author: kreyl
 */

#include "ir.h"
#include "uart.h"
//#include "main.h" // App is here

#if IR_TX_ENABLED // ========================== IR TX ==========================
//#define DBG_PINS
#ifdef DBG_PINS
#define DBG_GPIO1   GPIOC
#define DBG_PIN1    8
#define DBG_PIN_INIT()  PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull)
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
#else
#define DBG_PIN_INIT()
#define DBG1_SET()
#define DBG1_CLR()
#endif

namespace irLed {

#define IRLED_DMA_MODE  \
    STM32_DMA_CR_CHSEL(DAC_DMA_CHNL) | \
    DMA_PRIORITY_HIGH | \
    STM32_DMA_CR_MSIZE_BYTE | \
    STM32_DMA_CR_PSIZE_BYTE | \
    STM32_DMA_CR_MINC        | \
    STM32_DMA_CR_DIR_M2P | \
    STM32_DMA_CR_TCIE

Timer_t SamplingTmr{TMR_DAC_SMPL};
const stm32_dma_stream_t *PDmaTx = nullptr;
ftVoidVoid ICallbackI = nullptr;
static int32_t INRepeat;
static uint32_t TransactionSz;

#define SAMPLING_FREQ_HZ    (IR_CARRIER_HZ * 2)
// Every SamplePair contains 4 actual samples
#define NSAMPLES_HEADER     (((IR_HEADER_uS * IR_CARRIER_HZ) + 1999999UL) / 2000000UL)
#define NSAMPLES_SPACE      (((IR_SPACE_uS * IR_CARRIER_HZ) + 1999999UL) / 2000000UL)
#define NSAMPLES_ZERO       (((IR_ZERO_uS * IR_CARRIER_HZ) + 1999999UL) / 2000000UL)
#define NSAMPLES_ONE        (((IR_ONE_uS * IR_CARRIER_HZ) + 1999999UL) / 2000000UL)
#define NSAMPLES_PAUSE      (((IR_PAUSE_AFTER_uS * IR_CARRIER_HZ) + 1999999UL) / 2000000UL)

// DAC buf
#define DAC_BUF_SZ          (NSAMPLES_HEADER + NSAMPLES_SPACE + (NSAMPLES_ONE + NSAMPLES_SPACE) * IR_BIT_CNT + NSAMPLES_PAUSE)

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


static inline void StartTx() {
    dmaStreamSetMemory0(PDmaTx, DacBuf);
    dmaStreamSetTransactionSize(PDmaTx, TransactionSz);
    dmaStreamSetMode(PDmaTx, IRLED_DMA_MODE);
    dmaStreamEnable(PDmaTx);
    SamplingTmr.Enable();
}


void DmaTxEndIrqHandler(void *p, uint32_t flags) {
    chSysLockFromISR();
    SamplingTmr.Disable();
    dmaStreamDisable(PDmaTx);
    INRepeat--;
    if(INRepeat > 0) StartTx(); // Start over
    else if(ICallbackI) ICallbackI();
    chSysUnlockFromISR();
}

void Init() {
    DBG_PIN_INIT();
    // ==== GPIO ====
    // Once the DAC channel is enabled, the corresponding GPIO pin is automatically
    // connected to the DAC converter. In order to avoid parasitic consumption,
    // the GPIO pin should be configured in analog.
    PinSetupAnalog(IR_LED);
    // ==== DAC ====
    rccEnableDAC1();
    // Disable buffer
//    DAC->CR = 0;        // Disable DAC
//    DAC->MCR = 0b010;   // Disable buffer
    // Enable DAC, enable DMA, TIM7 TRGO evt as trigger, trigger enable
    DAC->CR = DAC_CR_EN1 | DAC_CR_DMAEN1 | (0b010 << 3) | DAC_CR_TEN1;
    // ==== DMA ====
    PDmaTx = dmaStreamAlloc(DAC_DMA, IRQ_PRIO_MEDIUM, DmaTxEndIrqHandler, nullptr);
    dmaStreamSetPeripheral(PDmaTx, &DAC->DHR8R1);
    // ==== Sampling timer ====
    SamplingTmr.Init();
    SamplingTmr.SetUpdateFrequencyChangingTopValue(SAMPLING_FREQ_HZ);
    SamplingTmr.SelectMasterMode(mmUpdate);
}

// Power is DAC value
void TransmitWord(uint16_t wData, uint8_t Power, int32_t NRepeat, ftVoidVoid CallbackI) {
    ICallbackI = CallbackI;
    INRepeat = NRepeat;
    // ==== Fill buffer depending on data ====
    DacSamplePair_t *p = DacBuf, ISampleCarrier{Power}, ISampleSpace{0};
    uint32_t i, j;
    // Put header
    for(i=0; i<NSAMPLES_HEADER; i++) *p++ = ISampleCarrier;
    for(i=0; i<NSAMPLES_SPACE; i++)  *p++ = ISampleSpace;
    // Put data
    for(j=0; j<IR_BIT_CNT; j++) {
        // Carrier
        if(wData & 0x8000) { for(i=0; i<NSAMPLES_ONE;  i++) *p++ = ISampleCarrier; }
        else               { for(i=0; i<NSAMPLES_ZERO; i++) *p++ = ISampleCarrier; }
        // Space
        for(i=0; i<NSAMPLES_SPACE; i++)  *p++ = ISampleSpace;
        wData <<= 1;
    }
    // Put pause
    for(i=0; i<NSAMPLES_PAUSE; i++) *p++ = ISampleSpace;

    // Debug print
//    for(uint8_t i=0; i<CHUNK_CNT; i++) Uart.Printf("%u %u\r", TxBuf[i].On, TxBuf[i].Duration);
//    Uart.Printf("\r");
    // ==== Start transmission ====
    TransactionSz = (p - DacBuf) * 4; // Every sample pair contains 4 actual samples
    StartTx()
    DBG1_SET();
}

void ResetI() {
    INRepeat = 0;
    dmaStreamDisable(PDmaTx);
    SamplingTmr.Disable();
    DAC->DHR8R1 = 0;
    SamplingTmr.GenerateUpdateEvt();
}
} // namespace
#endif

#if IR_RX_ENABLED // ========================== IR RX ==========================
irReceiver_t irRx;

static THD_WORKING_AREA(waIRRxThread, 128);
__noreturn
static void IRRxThread(void *arg) {
    chRegSetThreadName("IRRx");
    irRx.ITask();
}

void irReceiver_t::Init() {
    // GPIO
    PinSetupAlterFunc(IR_RCVR_PIN);
    /* ==== Timer ====
    ______        ________
          |______|
          ^      ^
       TI2FP2   TI2FP1
       Trigger  Capture
       Reset    CCR1 => DMA req CCR1 => TIMx Ch1 (not Ch2!) request
    */
    TmrRx.Init();
    TmrRx.SetTopValue(0xFFFF);        // Maximum
    TmrRx.SetupPrescaler(1000000);    // Input Freq: 1 MHz => one tick = 1 uS
    // Setup input capture mode for Channel2
    // Select TI2 as active input for CCR1
    TMR_IR_RX->CCMR1 = (0b10 << 0);
    // Select active polarity for TI2FP1 (capture CCR1) and TI2FP2 (trigger reset):
    // rising and falling edges, respectively (CC1P=0, CC2P=1). Look, TI2FP2 first, TI2FP1 second
    TMR_IR_RX->CCER = TIM_CCER_CC2P;
    // Select trigger input: TI2FP2 (TS = 110)
    TmrRx.SetTriggerInput(tiTI2FP2);
    // Configure slave mode controller in reset mode (SMS = 0100)
    TmrRx.SelectSlaveMode(smReset);
    // Enable the capture: CC1E = 1
    TMR_IR_RX->CCER |= TIM_CCER_CC1E;
    // DMA
    TmrRx.EnableDMAOnCapture(1);
    // ==== DMA ====
    dmaStreamAllocate     (IR_RX_TIM_DMA, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    dmaStreamSetPeripheral(IR_RX_TIM_DMA, &TMR_IR_RX->CCR1);
    dmaStreamSetMode      (IR_RX_TIM_DMA, IR_RX_DMA_MODE);
    dmaStreamSetMemory0   (IR_RX_TIM_DMA, IRxBuf);
    dmaStreamSetTransactionSize(IR_RX_TIM_DMA, IR_RX_BUF_LEN);
    dmaStreamEnable       (IR_RX_TIM_DMA);
    // ==== Start capture ====
    chThdCreateStatic(waIRRxThread, sizeof(waIRRxThread), NORMALPRIO, IRRxThread, NULL);
    TmrRx.Enable();
}

__noreturn
void irReceiver_t::ITask() {
    while(true) {
        chThdSleepMilliseconds(IR_RX_POLLING_PERIOD_MS);
        // Get number of bytes to process
#if defined STM32F2XX || defined STM32F4XX
        int32_t Sz = IR_RX_BUF_LEN - IR_RX_TIM_DMA->stream->NDTR;   // Number of bytes copied to buffer since restart
#else
        int32_t Sz = IR_RX_BUF_LEN - IR_RX_TIM_DMA->channel->CNDTR;   // Number of bytes copied to buffer since restart
#endif
        if(Sz != SzOld) {
            int32_t DurationCnt = Sz - SzOld;
            if(DurationCnt < 0) DurationCnt += IR_RX_BUF_LEN;   // Handle buffer circulation
            SzOld = Sz;
            // Iterate received bytes
            for(int32_t i=0; i<DurationCnt; i++) {
                uint16_t Dur = IRxBuf[RIndx++];
                if(RIndx >= IR_RX_BUF_LEN) RIndx = 0;
//                Uart.Printf("Dur: %u\r\n", Dur);
                // Reset pkt if delay is too large
                uint32_t Elapsed = ST2MS(chVTTimeElapsedSinceX(IPktStartTime));
                if(Elapsed > (2 * IR_RX_POLLING_PERIOD_MS)) {
                    IBitCnt = -1;
//                    Uart.Printf("IR TO %u\r", Elapsed);
                }
                // ==== Parse durations ====
                IrPktPartType_t PartType = MeasureDuration(Dur);
                switch(PartType) {
                    case iptHeader:
                        IBitCnt = 0;
                        IPktStartTime = chVTGetSystemTime();
                        ICurrentPkt.Word = 0;
                        break;
                    case iptOne:
                        if(IBitCnt >= 0) {
                            ICurrentPkt.Word <<= 1;
                            ICurrentPkt.Word |= 1;
                            IBitCnt++;
                        }
                        break;
                    case iptZero:
                        if(IBitCnt >= 0) {
                            ICurrentPkt.Word <<= 1;
                            IBitCnt++;
                        }
                        break;
                    case iptError:
                        IBitCnt = -1;   // Cancel pkt
//                        Uart.Printf("IR err %u\r", Dur);
                        break;
                } // switch
                // Check if Rx completed
                if(IBitCnt == IR_BIT_CNT) {
//                    Uart.Printf("IR RX: %04X\r\n", ICurrentPkt.Word);
                    IBitCnt = -1;
                    // Check CRC
                    if(ICurrentPkt.CalculateControlSum() == ICurrentPkt.ControlSum) { // all right
//                        Uart.Printf("ok\r");
                        LastPkt.Word = ICurrentPkt.Word; // Copy pkt
                        App.SignalEvt(EVT_IR_RX);
                    }
                } // if rx done
            } // for
        } // if sz
    } // while true
}
#endif
