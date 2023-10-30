/*
 * max98357.cpp
 *
 *  Created on: 18.09.2022 г.
 *      Author: layst
 */

#include "max98357.h"
#include "shell.h"


//    void DisableSAI() {
//        AU_SAI_A->CR1 &= ~SAI_xCR1_SAIEN;
//        while(AU_SAI_A->CR1 & SAI_xCR1_SAIEN); // wait until disabled
//    }
//    void FlushSAI()   { AU_SAI_A->CR2 = SAI_xCR2_FFLUSH; }


/*
The MAX98357A indicates the left channel word when LRCLK is low
Incoming serial data is always clocked-in on the rising edge of BCLK
LRCLK ONLY supports 8kHz, 16kHz, 32kHz, 44.1kHz, 48kHz, 88.2kHz and 96kHz
*/

#if 1 // =========================== SAI defins ================================
// Clock
struct I2SClkSetup_t {
    uint32_t Fs, Prediv1;
    Pll2Multi Pll2Mf;
    uint32_t Div, OF;
};

#if CRYSTAL_FREQ_HZ == 12000000UL
static const I2SClkSetup_t ClkSetup[] = {
        {.Fs= 8000, .Prediv1=2, .Pll2Mf=Pll2Multi::mul08, .Div=187, .OF=1},
        {.Fs=16000, .Prediv1=2, .Pll2Mf=Pll2Multi::mul16, .Div=187, .OF=1},
        {.Fs=32000, .Prediv1=3, .Pll2Mf=Pll2Multi::mul16, .Div= 62, .OF=1},
        {.Fs=44100, .Prediv1=2, .Pll2Mf=Pll2Multi::mul08, .Div= 34, .OF=0},
        {.Fs=48000, .Prediv1=2, .Pll2Mf=Pll2Multi::mul16, .Div= 62, .OF=1},
        {.Fs=88200, .Prediv1=2, .Pll2Mf=Pll2Multi::mul08, .Div= 17, .OF=0},
        {.Fs=96000, .Prediv1=5, .Pll2Mf=Pll2Multi::mul16, .Div= 12, .OF=1},
};
#else
#error "No freq divider setup for selected crystal"
#endif

#define I2S_DMATX_MODE  DMA_PRIO_HIGH | DMA_MEMSZ_16_BIT | DMA_PERSZ_16_BIT | DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_TCIE
#endif

// DMA Tx Completed IRQ
static void DmaTxEndIrqHandler(void *p, uint32_t flags);

static const DMA_t DmaTx {I2S_DMA_TX, DmaTxEndIrqHandler, nullptr, IRQ_PRIO_VERYHIGH};

static void DmaTxEndIrqHandler(void *p, uint32_t flags) {
//    Sys::LockFromIRQ();
    if(Codec::I2SDmaDoneCbI) Codec::I2SDmaDoneCbI();
//    Sys::UnlockFromIRQ();
}

namespace Codec {

ftVoidVoid I2SDmaDoneCbI = nullptr;

void Init() {
    // === GPIOs ===
    Gpio::SetupAlterFunc(AU_LRCK, Gpio::PushPull); // Left/Right (Frame sync) clock output
    Gpio::SetupAlterFunc(AU_BCLK, Gpio::PushPull); // Bit clock output
    Gpio::SetupAlterFunc(AU_SDIN, Gpio::PushPull); // SAI_A is Master Transmitter
    Gpio::SetupOut(AU_SDMODE, Gpio::PushPull);
    Gpio::SetHi(AU_SDMODE); // Enable device, use Left channel
    Sys::SleepMilliseconds(18);
    // === Clock ===
    RCU->EnSpi(AU_SPI);
    // Use PLL2 as I2S clock
    RCU->CFG1 |= RCU_CFG1_I2S1SEL | RCU_CFG1_I2S2SEL; // Set both. Even if one is unused - who cares.
    // === Setup I2S as Master Transmitter ===
    AU_SPI->CTL0 = 0; // Disable SPI
    // ChnlLen 16bit, chnl payload data 16bit, Clk Idle Low, I2S std, Master TX, use I2S not SPI
    AU_SPI->I2SCTL = I2S_CTL_CHLEN_16bit | I2S_CTL_DTLEN_16bit | I2S_CTL_CKPL_IdleLo |
            I2S_CTL_STD_I2S | I2S_CTL_MASTER_TX | I2S_CTL_I2SSEL;
    // ==== DMA ====
    AU_SPI->EnTxDma();
    DmaTx.Init(&AU_SPI->DATA, I2S_DMATX_MODE);
}

void TransmitBuf(void *Buf, uint32_t Sz16) {
    DmaTx.Disable();
    DmaTx.SetMemoryAddr(Buf);
    DmaTx.SetTransferDataCnt(Sz16);
    DmaTx.Enable();
    AU_SPI->EnableI2S(); // Start tx
}

bool IsTransmitting() {
    return (DmaTx.GetTransferDataCnt() != 0);
}

void Stop() {
    DmaTx.Disable();
    AU_SPI->DisableI2S();
    RCU->DisablePll2();
}

// supports 8kHz, 16kHz, 32kHz, 44.1kHz, 48kHz, 88.2kHz and 96kHz only
retv SetupSampleRate(uint32_t Fs) {
    Stop();
    PrintfI("Fs: %u\r", Fs);
    for(auto& stp : ClkSetup) {
        if(stp.Fs == Fs) { // setup for Fs found, apply it
            RCU->SetPrediv1(stp.Prediv1);
            RCU->SetPll2Multi(stp.Pll2Mf);
            AU_SPI->I2SPSC = (stp.OF << 8) | stp.Div;
            if(RCU->EnablePll2() == retv::Ok) return retv::Ok;
            else {
                PrintfI("I2S PLL fail\r");
                return retv::Fail;
            }
        }
    }
    PrintfI("Fs %u not supported\r", Fs);
    return retv::Fail;
}

} // namespace

#if 0 // ============================== IRQ ====================================
extern "C"
OSAL_IRQ_HANDLER(SAI_IRQ_HANDLER) {
    OSAL_IRQ_PROLOGUE();
    if(AU_SAI_A->SR & SAI_xSR_FREQ) {
        AU_SAI_A->DR = 0x0001;
        AU_SAI_A->DR = 0x0003;
    }
    OSAL_IRQ_EPILOGUE();
}
#endif
