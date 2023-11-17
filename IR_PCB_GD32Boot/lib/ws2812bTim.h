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
#define NPX_PARAMS      PA0, TIM4, 0
#define NPX_PWR_EN      GPIOC, 0
// Npx LEDs
#define NPX_DMA             DMA1_Channel1 // Tim4 Update

static const NpxParams NParams{NPX_PARAMS, NPX_DMA, NPX_LED_CNT, NpxParams::ClrType::RGB};
Neopixels_t Leds{&NpxParams};
*/

#include "gd_lib.h"
#include "color.h"
#include "board.h"
#include <vector>

#define TICK_DUR_ns     20UL    // For 50MHz. Tune this.

typedef std::vector<Color_t> ColorBuf_t;

struct NpxParams {
    enum class ClrType {RGB=24UL, RGBW=32UL}; // RGB is 3 bytes = 24bits, RGBW is 4bytes = 32bits
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
    uint32_t IBitBufCnt = 0;
    uint16_t *IBitBuf = nullptr;
    HwTim Tim;
    DMA_t DmaTx;
    void OnDmaDone();
    friend void NpxDmaDone(void *p, uint32_t flags);
public:
    bool TransmitDone = false;
    ftVoidVoid OnTransmitEnd = nullptr;
    Neopixels_t(const NpxParams *APParams);
    void SetCurrentColors();
    ColorBuf_t ClrBuf;
    void Init();
    void SetAll(Color_t Clr) { std::fill(ClrBuf.begin(), ClrBuf.end(), Clr); }
    bool AreOff() {
        for(auto &IClr : ClrBuf) if(IClr != clBlack) return false;
        return true;
    }
};
