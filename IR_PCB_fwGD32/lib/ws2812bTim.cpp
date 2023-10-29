#include "ws2812bTim.h"

void NpxDmaDone(void *p, uint32_t flags) {
    ((Neopixels_t*)p)->OnDmaDone();
}

Neopixels_t::Neopixels_t(const NpxParams *APParams) :
    Params(APParams), Tim(APParams->PTim),
    DmaTx(APParams->DmaChnlTx, NpxDmaDone, this) {}


void Neopixels_t::Init() {
    Gpio::SetupAlterFunc(Params->PGpio, Params->Pin, Gpio::PushPull);
    Tim.Init();
    Tim.SetInputFreqChangingPrescaler(2500000);



    // Allocate memory
    ClrBuf.resize(Params->NpxCnt);
//    IBitBufSz = NPX_RST_BYTE_CNT + (Params->NpxCnt * (Params->Type == npxRGBW? 4 : 3) * NPX_BYTES_PER_BYTE);
//    Printf("LedCnt: %u; TotalByteCnt: %u\r", Params->NpxCnt, IBitBufSz);
//    IBitBuf = (uint8_t*)malloc(IBitBufSz);
//    memset(IBitBuf, 0, IBitBufSz); // Zero it all, to zero head and tail

    // ==== DMA ====
//    PDma = dmaStreamAlloc(Params->DmaID, IRQ_PRIO_LOW, NpxDmaDone, this);
//    dmaStreamSetPeripheral(PDma, &Params->ISpi.PSpi->DR);
//    dmaStreamSetMode      (PDma, Params->DmaMode);
    TransmitDone = true;
}

void Neopixels_t::SetCurrentColors() {
    /*
    TransmitDone = false;
#if WS2812B_V2
    uint8_t *p = IBitBuf + (NPX_RST_BYTE_CNT / 2); // First and last words are zero to form reset
    // Fill bit buffer
    if(Params->Type == npxRGBW) {
        for(auto &Color : ClrBuf) {
            memcpy(p, &ITable[Color.G], 4);
            p += NPX_BYTES_PER_BYTE;
            memcpy(p, &ITable[Color.R], 4);
            p += NPX_BYTES_PER_BYTE;
            memcpy(p, &ITable[Color.B], 4);
            p += NPX_BYTES_PER_BYTE;
            memcpy(p, &ITable[Color.W], 4);
            p += NPX_BYTES_PER_BYTE;
        }
    }
    else {
        for(auto &Color : ClrBuf) {
            memcpy(p, &ITable[Color.G], 4);
            p += NPX_BYTES_PER_BYTE;
            memcpy(p, &ITable[Color.R], 4);
            p += NPX_BYTES_PER_BYTE;
            memcpy(p, &ITable[Color.B], 4);
            p += NPX_BYTES_PER_BYTE;
        }
    }
#else // WS2812B_V5 and SK6812SIDE
    // Fill bit buffer
    uint32_t *p = (uint32_t*)IBitBuf + (NPX_RST_BYTE_CNT / 8); // First and last words are zero to form reset
    if(Params->Type == npxRGBW) {
        for(auto &Color : ClrBuf) {
            *p++ = ITable[Color.G];
            *p++ = ITable[Color.R];
            *p++ = ITable[Color.B];
            *p++ = ITable[Color.W];
        }
    }
    else {
        for(auto &Color : ClrBuf) {
            *p++ = ITable[Color.G];
            *p++ = ITable[Color.R];
            *p++ = ITable[Color.B];
        }
    }
#endif
    // Start transmission
    dmaStreamDisable(PDma);
    dmaStreamSetMemory0(PDma, IBitBuf);
    dmaStreamSetTransactionSize(PDma, IBitBufSz);
    dmaStreamSetMode(PDma, Params->DmaMode);
    dmaStreamEnable(PDma);
*/
}


void Neopixels_t::OnDmaDone() {
    TransmitDone = true;
    if(OnTransmitEnd) OnTransmitEnd();
}


