/*
 * beeper.h
 *
 *  Created on: 22 ����� 2015 �.
 *      Author: Kreyl
 */

#ifndef BEEPER_H__
#define BEEPER_H__

#include "BaseSequencer.h"
#include "gd_lib.h"

class Beeper : public BaseSequencer<BeepChunk> {
private:
    const PinOutputPWM_t IPin;
    uint32_t CurrFreq = 0;
    void ISwitchOff() { IPin.Set(0); }
    SequencerLoopTask_t ISetup() {
        IPin.Set(pcurrent_chunk->volume);
        if(pcurrent_chunk->freq_smooth == 0) { // If smooth time is zero, set now
            CurrFreq = pcurrent_chunk->freq_Hz;
            IPin.SetFrequencyHz(CurrFreq);
            pcurrent_chunk++;   // goto next
        }
        else {
            if     (CurrFreq < pcurrent_chunk->freq_Hz) CurrFreq += 100;
            else if(CurrFreq > pcurrent_chunk->freq_Hz) CurrFreq -= 100;
            IPin.SetFrequencyHz(CurrFreq);
            // Check if completed now
            if(CurrFreq == pcurrent_chunk->freq_Hz) pcurrent_chunk++;
            else { // Not completed
                // Calculate time to next adjustment
                uint32_t Delay = (pcurrent_chunk->freq_smooth / (CurrFreq+4)) + 1;
                SetupDelay(Delay);
                return sltBreak;
            } // Not completed
        }
        return sltProceed;
    }
public:
    Beeper(const PwmSetup_t APinSetup) : BaseSequencer(), IPin(APinSetup) {}
    void Init() { IPin.Init(); }
    void Beep(uint32_t Freq_Hz, uint8_t Volume) {
        IPin.SetFrequencyHz(Freq_Hz);
        IPin.Set(Volume);
    }
    void Off() { IPin.Set(0); }
};

#endif //BEEPER_H__
