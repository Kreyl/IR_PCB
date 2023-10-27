/*
 * ChunkTypes.h
 *
 *  Created on: 08 Jan 2015
 *      Author: Kreyl
 */

#ifndef CHUNK_TYPES_H_
#define CHUNK_TYPES_H_

#include "color.h"
#include "MsgQ.h"

enum class Chunk {Setup, Wait, Goto, End, Repeat};

// ==== Different types of chunks ====
struct BaseChunk_t { //  Everyone must contain this.
    Chunk ChunkSort;
    union {
        uint32_t Value;
        uint32_t Volume;
        uint32_t Time_ms;
        uint32_t ChunkToJumpTo;
        int32_t RepeatCnt;
    };
};

// RGB LED chunk
struct LedRGBChunk_t : public BaseChunk_t {
    Color_t Color;
};

// HSV LED chunk
struct LedHSVChunk_t : public BaseChunk_t {
    ColorHSV_t Color;
};

// LED Smooth
struct LedSmoothChunk_t : public BaseChunk_t {
    uint32_t Brightness;
};

// Beeper
struct BeepChunk_t : public BaseChunk_t {
    uint32_t Freq_Hz;
    uint32_t FreqSmooth = 0;
};


#if 1 // ====================== Base sequencer class ===========================
void VTmrUniversalCb(void* p);

template <class TChunk>
class BaseSequencer_t : private IrqHandler_t {
protected:
    enum SequencerLoopTask_t {sltProceed, sltBreak};
    VirtualTimer_t ITmr;
    const TChunk *IPStartChunk, *IPCurrentChunk;
    int32_t RepeatCounter = -1;
    EvtMsg_t IEvtMsg;
    virtual void ISwitchOff() = 0;
    virtual SequencerLoopTask_t ISetup() = 0;
//    friend void VTmrUniversalCb(void* p);
    void SetupDelay(uint32_t ms) { ITmr.SetI(TIME_MS2I(ms), VTmrUniversalCb, this); }

    // Process sequence
    void IIrqHandlerI() {
        if(ITmr.IsArmedX()) ITmr.ResetI();  // Reset timer
        while(true) {   // Process the sequence
            switch(IPCurrentChunk->ChunkSort) {
                case Chunk::Setup: // setup now and exit if required
                    if(ISetup() == sltBreak) return;
                    break;

                case Chunk::Wait: { // Start timer, pointing to next chunk
                        uint32_t Delay = IPCurrentChunk->Time_ms;
                        IPCurrentChunk++;
                        if(Delay != 0) {
                            SetupDelay(Delay);
                            return;
                        }
                    }
                    break;

                case Chunk::Goto:
                    IPCurrentChunk = IPStartChunk + IPCurrentChunk->ChunkToJumpTo;
                    if(IEvtMsg.ID != EvtId::None) EvtQMain.SendNowOrExitI(IEvtMsg);
                    SetupDelay(1);
                    return;
                    break;

                case Chunk::End:
                    if(IEvtMsg.ID != EvtId::None) EvtQMain.SendNowOrExitI(IEvtMsg);
                    IPStartChunk = nullptr;
                    IPCurrentChunk = nullptr;
                    return;
                    break;

                case Chunk::Repeat:
                    if(RepeatCounter == -1) RepeatCounter = IPCurrentChunk->RepeatCnt;
                    if(RepeatCounter == 0) {    // All was repeated, goto next
                        RepeatCounter = -1;     // reset counter
                        IPCurrentChunk++;
                    }
                    else {  // repeating in progress
                        IPCurrentChunk = IPStartChunk;  // Always from beginning
                        RepeatCounter--;
                    }
                    break;
            } // switch
        } // while
    } // IProcessSequenceI
public:
    void SetupSeqEndEvt(EvtMsg_t AEvtMsg) { IEvtMsg = AEvtMsg; }

    void StartOrRestart(const TChunk *PChunk) {
        Sys::Lock();
        RepeatCounter = -1;
        IPStartChunk = PChunk;   // Save first chunk
        IPCurrentChunk = PChunk;
        IIrqHandlerI();
        Sys::Unlock();
    }

    void StartOrContinue(const TChunk *PChunk) {
        if(PChunk == IPStartChunk) return; // Same sequence
        else StartOrRestart(PChunk);
    }

    void Stop() {
        if(IPStartChunk != nullptr) {
            Sys::Lock();
            if(ITmr.IsArmedX()) ITmr.ResetI();
            IPStartChunk = nullptr;
            IPCurrentChunk = nullptr;
            Sys::Unlock();
        }
        ISwitchOff();
    }
    const TChunk* GetCurrentSequence() { return IPStartChunk; }
    bool IsIdle() { return (IPStartChunk == nullptr and IPCurrentChunk == nullptr); }
};
#endif

#endif // CHUNK_TYPES_H_
