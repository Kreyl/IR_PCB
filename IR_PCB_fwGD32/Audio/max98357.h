/*
 * max98357.h
 *
 *  Created on: 18 сент. 2022 г.
 *      Author: layst
 */

#ifndef __MAX98357_H
#define __MAX98357_H

#include "gd_lib.h"

// Beware: PLL2 is used for I2S clk generation, use Crystal to feed PLL2, not IRC48M

struct StereoSample_t {
    int16_t Left, Right;
} __attribute__((packed));

namespace Codec {

extern ftVoidVoid I2SDmaDoneCbI;
void Init();

// supports 8kHz, 16kHz, 32kHz, 44.1kHz, 48kHz, 88.2kHz and 96kHz only
retv SetupSampleRate(uint32_t Fs);
//    void Put(uint16_t Data) { AU_SAI_A->DR = Data; }

void TransmitBuf(void *Buf, uint32_t Sz16);
bool IsTransmitting();
void Stop();
};

#endif // __MAX98357_H
