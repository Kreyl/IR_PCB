#include "ws2812bTim.h"
#include "shell.h"

#define NPX_DMA_MODE  DMA_PRIO_VERYHIGH | DMA_MEMSZ_16_BIT | DMA_PERSZ_16_BIT | DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_TCIE

#if WS2812B_V2
#define ONE_DUR_ns      850UL
#define ZERO_DUR_ns     400UL

#define ONE_DUR_ticks   (ONE_DUR_ns / TICK_DUR_ns)
#define ZERO_DUR_ticks  (ZERO_DUR_ns / TICK_DUR_ns)
#define TIM_TOP_ticks   (ONE_DUR_ticks + ZERO_DUR_ticks)

#define RESET_BITS_CNT  240UL   // 300us / 1.260us per bit
#else

#endif

void NpxDmaDone(void *p, uint32_t flags) { ((Neopixels_t*)p)->OnDmaDone(); }

Neopixels_t::Neopixels_t(const NpxParams *APParams) :
    Params(APParams), Tim(APParams->PTim),
    DmaTx(APParams->DmaChnlTx, NpxDmaDone, this) {}

void Neopixels_t::Init() {
    gpio::SetupAlterFunc(Params->PGpio, Params->Pin, gpio::PushPull, gpio::speed50MHz);
    Tim.Init();
    Tim.SetTopValue(TIM_TOP_ticks);
//    Tim.SetInputFreqChangingPrescaler(2500000);
    // Setup output in PWM mode
    Tim.EnPrimaryOutput();
    Tim.SetChnlMode(Params->TimChnl, HwTim::ChnlMode::Output);
    Tim.SetOutputCmpMode(Params->TimChnl, HwTim::CmpMode::PWM0HiLo);
    Tim.EnableOutputShadow(Params->TimChnl);
    Tim.EnChnl(Params->TimChnl);
    Tim.EnableDmaOnUpdate();
    // Allocate memory
    ClrBuf.resize(Params->NpxCnt);
    IBitBufCnt = RESET_BITS_CNT + (Params->NpxCnt * (uint32_t)Params->Type);
    Printf("LedCnt: %u; BitBufCnt: %u\r", Params->NpxCnt, IBitBufCnt);
    IBitBuf = (uint16_t*)malloc(IBitBufCnt * sizeof(uint16_t));
    for(uint32_t i=0; i<IBitBufCnt; i++) IBitBuf[i] = 0; // Zero it all, to zero head and tail
    // ==== DMA ====
    DmaTx.Init(Tim.GetChnlRegAddr(Params->TimChnl), NPX_DMA_MODE);
    TransmitDone = true;
}

__attribute__((always_inline))
static inline void PutBits(uint16_t **ptr, uint8_t byte) {
    uint16_t *p = *ptr;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    byte <<= 1UL;
    *p++ = (byte & 0x80)? ONE_DUR_ticks : ZERO_DUR_ticks;
    *ptr = p;
}

void Neopixels_t::SetCurrentColors() {
    TransmitDone = false;
    // Fill bit buffer
    uint16_t *p = IBitBuf + (RESET_BITS_CNT / 2); // First and last bits are zero to form reset
    if(Params->Type == NpxParams::ClrType::RGB) {
        for(auto &Color : ClrBuf) {
            PutBits(&p, Color.G);
            PutBits(&p, Color.R);
            PutBits(&p, Color.B);
        }
    }
    else {
        for(auto &Color : ClrBuf) {
            PutBits(&p, Color.G);
            PutBits(&p, Color.R);
            PutBits(&p, Color.B);
            PutBits(&p, Color.W);
        }
    }
    // Start transmission
    Tim.Disable();
    DmaTx.Disable();
    DmaTx.SetMemoryAddr(IBitBuf);
    DmaTx.SetTransferDataCnt(IBitBufCnt);
    DmaTx.Enable();
    Tim.Enable();
}

void Neopixels_t::OnDmaDone() {
    Tim.Disable();
    DmaTx.Disable();
    TransmitDone = true;
    if(OnTransmitEnd) OnTransmitEnd();
}
