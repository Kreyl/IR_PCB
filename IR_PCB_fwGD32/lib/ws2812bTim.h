#pragma once

/*
 * ========== WS2812 control module ==========
 * Only basic command "SetCurrentColors" is implemented, all other is up to
 * higher level software.
 * There are different timings for V2 and V5. Wir verachten sie.
 */
#define WS2812B_V2    TRUE
//#define WS2812B_V5    TRUE  // (and SK6812SIDE)

/*
 * WS2812 V2 requires timings, ns: (400 + 850) +- 150 each.
 * Reset must be:
 *    WS2812: 50uS => 125bit => ~16 bytes
 *    WS2813: 300uS => 750bit => 94 bytes
 *
 * WS2812 V5 requires timings, ns:
 * 0 is [220;380]+[580;1000]
 * 1 is [580;1000]+[580;1000]
 *
 * SK6812-SIDE (4020) requires timings, ns:
 * 0 is [200;400(typ 320)]+[800;...]
 * 1 is [620;1000(typ 640)]+[200;...]
 * Reset must be: 80uS => ~248bit => ~31 bytes
 *
== EXAMPLE ==
#define NPX_LED_CNT     (14*3)
#define NPX_SPI         SPI2
#define NPX_DATA_PIN    GPIOC, 3, AF1
#define NPX_PWR_EN      GPIOC, 0

static const NeopixelParams_t NpxParams{NPX_SPI, NPX_DATA_PIN, LEDWS_DMA, NPX_DMA_MODE(0), NPX_LED_CNT, npxRGBW};
Neopixels_t Leds{&NpxParams};
 */

#include "gd_lib.h"
#include "color.h"
#include "board.h"
#include <vector>

typedef std::vector<Color_t> ColorBuf_t;

// SPI Buffer (no tuning required)
#if WS2812B_V2
//#define NPX_SPI_BITRATE         2500000
//#define NPX_SPI_BITNUMBER       bitn8
//#define NPX_BYTES_PER_BYTE      3 // 3 bits of SPI to produce 1 bit of LED data
//#define NPX_RST_BYTE_CNT        100

//#define NPX_DMA_MODE(Chnl) \
//                        (STM32_DMA_CR_CHSEL(Chnl) \
//                        | DMA_PRIORITY_HIGH \
//                        | STM32_DMA_CR_MSIZE_BYTE \
//                        | STM32_DMA_CR_PSIZE_BYTE \
//                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
//                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
//                        | STM32_DMA_CR_TCIE

#else // WS2812B_V5 and SK6812SIDE
#define NPX_SPI_BITRATE         3000000
#define NPX_SPI_BITNUMBER       bitn16
#define NPX_BYTES_PER_BYTE      4 // 2 bits are 1 byte, 8 bits are 4 bytes
#define NPX_RST_BYTE_CNT        108

#define NPX_DMA_MODE(Chnl) \
                        (STM32_DMA_CR_CHSEL(Chnl) \
                        | DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P)  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE
#endif


void NpxPrintTable();



struct NpxParams {
    enum class ClrType {RGB, RGBW};
    GPIO_TypeDef *PGpio;
    uint16_t Pin;
    TIM_TypeDef *PTim;
    uint32_t TimChnl;
    DMAChannel_t *DmaChnlTx;
    uint32_t NpxCnt;
    ClrType Type;
    NpxParams(
            GPIO_TypeDef *APGpio, uint32_t APin,
            TIM_TypeDef *APTim, uint32_t ATimChnl,
            DMAChannel_t *ADmaChnlTx,
            uint32_t NpxCnt, ClrType AType) :
                PGpio(APGpio), Pin(APin), PTim(APTim), TimChnl(ATimChnl),
                DmaChnlTx(ADmaChnlTx), NpxCnt(NpxCnt), Type(AType) {}
};


class Neopixels_t {
private:
    const NpxParams *Params;
    uint32_t IBitBufSz = 0;
    uint8_t *IBitBuf = nullptr;
    HwTim Tim;
    DMA_t DmaTx;
public:
    bool TransmitDone = false;
    ftVoidVoid OnTransmitEnd = nullptr;
    Neopixels_t(const NpxParams *APParams);
    void SetCurrentColors();
    void OnDmaDone();
    ColorBuf_t ClrBuf;
    void Init();
    void SetAll(Color_t Clr) { std::fill(ClrBuf.begin(), ClrBuf.end(), Clr); }
    bool AreOff() {
        for(auto &IClr : ClrBuf) if(IClr != clBlack) return false;
        return true;
    }
};
