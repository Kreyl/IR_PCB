/*
 * app.cpp
 *
 *  Created on: 14.08.2023
 *      Author: layst
 */

#include "app.h"
#include "Settings.h"
#include "gd_lib.h"
#include "shell.h"
#include "ir.h"
#include "ir_pkt.h"
#include "MsgQ.h"
#include "led.h"
#include "beeper.h"
#include "Sequences.h"

int32_t hit_cnt, rounds_cnt, magazines_cnt;
extern Beeper beeper;
extern bool is_testing;

#if 1 // ========================== Message queue ==============================
enum class AppEvt : uint16_t {
    Reset,
    StartFire, EndOfIrTx, EndOfDelayBetweenShots, MagazineReloadDone,
    IrRx,
    NewPwmData,
};

union AppMsg {
    uint32_t dword;
    struct {
        uint16_t data16;
        AppEvt evt;
    } __attribute__((__packed__));

    AppMsg& operator = (const AppMsg &Right) {
        dword = Right.dword;
        return *this;
    }
    AppMsg() : dword(0) {}
    AppMsg(AppEvt aevt) : data16(0), evt(aevt) {}
    AppMsg(AppEvt aevt, uint16_t data) : data16(data), evt(aevt) {}
} __attribute__((__packed__));

static EvtMsgQ<AppMsg, 9> evt_q_app;
#endif

#if 1 // ============================== Controls ===============================
// ==== Outputs ====
void PulserCallback(void *p);

class PulserPin: private Pin_t {
private:
    VirtualTimer itmr;
public:
    PulserPin(GPIO_TypeDef *APGPIO, uint16_t APin) : Pin_t(APGPIO, APin) {}
    void Init() { Pin_t::SetupOut(Gpio::PushPull); }
    void PulseI(uint32_t Dur) {
        itmr.ResetI();
        SetHi();
        if(Dur == 0) SetLo();
        else itmr.SetI(TIME_MS2I(Dur), PulserCallback, (void*)this);
    }
    void ResetI() {
        itmr.ResetI();
        SetLo();
    }
    void SetHi() { Pin_t::SetHi(); }
    void SetLo() { Pin_t::SetLo(); }

    // Inner use
    void IOnTmrDone() { SetLo(); }
};

void PulserCallback(void *p) { static_cast<PulserPin*>(p)->IOnTmrDone(); }

PulserPin output_hits_ended {Output_HitsEnded};

// ==== Inputs ====
void InputPinIrqHandlerI(); // Common handler for both inputs
void InputTmrCallback(void *p) { __NOP(); }

class InputPin : public PinIrq {
public:
    VirtualTimer itmr;
    InputPin(GPIO_TypeDef *apgpio, uint16_t apin_n, Gpio::PullUpDown apull_up_down) :
        PinIrq(apgpio, apin_n, apull_up_down, InputPinIrqHandlerI) {}
    void Init() {
        PinIrq::Init(ttRising);
        CleanIrqFlag();
        EnableIrq(IRQ_PRIO_LOW);
    }
    bool CheckIfProcess() {
        if(itmr.IsArmedX()) return false; // Not enough time passed
        else {
            itmr.SetI(TIME_MS2I(INPUT_DEADTIME_ms), InputTmrCallback, nullptr);
            return true;
        }
    }
};

InputPin input_single_fire{Input_SingleFire};
InputPin input_burst_fire {Input_BurstFire};

void InputPinIrqHandlerI() {
    uint32_t flags = EXTI->PD;
    // input for single fire
    if(flags & (1 << input_single_fire.pin_n)) {
        if(input_single_fire.CheckIfProcess())
            evt_q_app.SendNowOrExitI(AppEvt::StartFire);
    }
    // input for burst fire
    if(flags & (1 << input_burst_fire.pin_n)) {
        if(input_burst_fire.CheckIfProcess())
            evt_q_app.SendNowOrExitI(AppEvt::StartFire);
    }
}

/* Here Input0 is used for restarting and capturing on the rising edge
and capturing on the falling one. Two outputs of EdgeDetector1 are used: CI0FE0 and CI0FE1.
          _________
   ______|         |________
         ^         ^
Input    CI0       CI1
EdgeOut: CI0FE0    CI0FE1
Capture: period    pulse width
         Trigger
         Restart
*/

void PwmInputTimCallback(void *p);

class PwmInputPin : private HwTim {
private:
    GPIO_TypeDef *pgpio;
    uint16_t pin_n;
    Gpio::PullUpDown pull_up_down;
    VirtualTimer vtmr;
    friend void PwmInputTimCallback(void *p);

    void TimerCallback() {
        Sys::LockFromIRQ(); // The callback is invoked outside the kernel critical zone
        // Are there new values?
        bool ch0_capture_occured = HwTim::itmr->INTF & TIM_INTF_CH0IF;
        bool ch1_capture_occured = HwTim::itmr->INTF & TIM_INTF_CH1IF;
        if(!ch0_capture_occured and !ch1_capture_occured) { // No new values
            if(pwm_duty != 0) { // There was an impulse before and now it has gone
                pwm_duty = 0; // Means no pulses
                evt_q_app.SendNowOrExitI(AppMsg(AppEvt::NewPwmData));
            }
        }
        // Both width and period are captured. Ignore if only one value is done.
        else if(ch0_capture_occured and ch1_capture_occured) {
            // Calc new duty
            int32_t period = HwTim::itmr->CH0CV;
            int32_t pulse_width = HwTim::itmr->CH1CV;
            int32_t new_pwm_duty = (period == 0)? 0 : (100UL * pulse_width) / period;
            // Check if changed
            int32_t diff = pwm_duty - new_pwm_duty;
            if(diff < 0) diff = -diff;
            if(diff > PWM_DUTY_DEVIATION_percent) {
                pwm_duty = new_pwm_duty;
                evt_q_app.SendNowOrExitI(AppMsg(AppEvt::NewPwmData));
            }
        }
        // Restart timer
        vtmr.SetI(TIME_MS2I(INPUT_PWM_CHECK_PERIOD_ms), PwmInputTimCallback, this);
        Sys::UnlockFromIRQ();
    }
public:
    volatile int32_t pwm_duty=0;

    PwmInputPin(GPIO_TypeDef *apgpio, uint16_t apin_n, Gpio::PullUpDown apull_up_down,
            TIM_TypeDef *aptimer) : HwTim(aptimer),
                pgpio(apgpio), pin_n(apin_n), pull_up_down(apull_up_down) {}

    void Init() {
        Gpio::SetupInput(pgpio, pin_n, pull_up_down);
        HwTim::Init();
        HwTim::SetTopValue(0xFFFF); // Maximum
        // === Input0 === on the rising edge, perform a capture and restart
        HwTim::SetInputActiveEdge(0, RiseFall::Rising); // CI0FE0 is Active Rising (CI1FE0 also, but not used)
        // Setup input mode for Channel0: capture Input0 (IS0 = CI0FE0)
        HwTim::SetChnlMode(0, HwTim::ChnlMode::CI0FE0); // Chnl0 is input, capture on Input0's CI0FE0 signal
        // Restart timer on trigger; trigger is CI0FE0
        HwTim::SetTriggerInput(HwTim::TriggerIn::CI0FE0); // Use Input0's CI0FE0 as TriggerIn
        HwTim::SelectSlaveMode(HwTim::SlaveMode::Restart); // Configure slave mode controller in Restart mode
        HwTim::EnChnl(0); // Enable capture on channel 0
        // === Input1 === on the falling edge, perform capture
        HwTim::SetInputActiveEdge(1, RiseFall::Falling); // CI0FE1 is Active Falling (CI1FE1 also, but not used)
        // Setup input mode for Channel1: capture Input0 (IS1 = CI0FE1)
        HwTim::SetChnlMode(1, HwTim::ChnlMode::CI0FE1); // Chnl0 is input, capture on Input0's CI0FE1 signal
        HwTim::EnChnl(1); // Enable capture on channel 1
        // === Start Timer ===
        HwTim::Enable();
        vtmr.Set(TIME_MS2I(INPUT_PWM_CHECK_PERIOD_ms), PwmInputTimCallback, this);
    }
};
PwmInputPin input_pwm { INPUT_PWM };

void PwmInputTimCallback(void *p) { static_cast<PwmInputPin*>(p)->TimerCallback(); }

// For Debug
static uint32_t iinputs[2];
void SetInputs(uint32_t ainputs[2]) {
    if(iinputs[0] == 0 and ainputs[0] == 1) evt_q_app.SendNowOrExit(AppEvt::StartFire);
    if(iinputs[1] == 0 and ainputs[1] == 1) evt_q_app.SendNowOrExit(AppEvt::StartFire);
    iinputs[0] = ainputs[0];
    iinputs[1] = ainputs[1];
}

bool DoBurstFire() {
    return (input_pwm.pwm_duty >= PWM_DUTY_BURST_FIRE_percent) or
            input_burst_fire.IsHi() or
            iinputs[0] == 1;
}
#endif

#if 1 // =========================== Indication ================================
extern LedSmooth side_LEDs[SIDE_LEDS_CNT];
extern LedSmooth front_LEDs[FRONT_LEDS_CNT];

#define SIDE_LEDS_ENABLED   TRUE

namespace Indication {

enum class IndiState { Idle, Reloading, MagazinesEnded, HitsEnded };
static IndiState state = IndiState::Idle;

void Shot() {
    beeper.StartOrRestart(bsqShot);
    for(auto& led : front_LEDs) led.StartOrRestart(lsqShot);
    Printf("#Shot; %d/%d left\r\n", rounds_cnt, magazines_cnt);
}

void RoundsEnded() {
    Sys::Lock();
    // Beeper
    if(beeper.GetCurrentSequence() == bsqHit) beeper.SetNextSequenceI(bsqReloading);
    else beeper.StartOrRestartI(bsqReloading);
    // Leds
#if SIDE_LEDS_ENABLED
    if(side_LEDs[3].GetCurrentSequence() == lsqHit) side_LEDs[3].SetNextSequenceI(lsqReloading);
    else {
        side_LEDs[0].StopI();
        side_LEDs[1].StopI();
        side_LEDs[2].StopI();
        side_LEDs[3].StartOrRestartI(lsqReloading);
    }
#endif
    state = IndiState::Reloading;
    Sys::Unlock();
    Printf("#RoundsEnded\r\n");
}

void MagazineReloaded() {
    Sys::Lock();
    // Beeper
    if(beeper.GetCurrentSequence() == bsqHit) beeper.SetNextSequenceI(bsqMagazReloaded);
    else beeper.StartOrRestartI(bsqMagazReloaded);
    // Leds
#if SIDE_LEDS_ENABLED
    if(side_LEDs[3].GetCurrentSequence() != lsqHit) {
        for(auto& Led : side_LEDs) Led.StopI();
    }
#endif
    state = IndiState::Idle;
    Sys::Unlock();
    Printf("#MagazineReloaded\r\n");
}

void MagazinesEnded() {
    Sys::Lock();
    // Beeper
    if(beeper.GetCurrentSequence() == bsqHit) beeper.SetNextSequenceI(bsqMagazEnded);
    else beeper.StartOrRestartI(bsqMagazEnded);
    // Leds
#if SIDE_LEDS_ENABLED
    if(side_LEDs[3].GetCurrentSequence() == lsqHit)
        side_LEDs[3].SetNextSequenceI(lsqMagazinesEnded);
    else {
        side_LEDs[0].StopI();
        side_LEDs[1].StopI();
        side_LEDs[2].StopI();
        side_LEDs[3].StartOrRestartI(lsqMagazinesEnded);
    }
#endif
    state = IndiState::MagazinesEnded;
    Sys::Unlock();
    Printf("#MagazinesEnded\r\n");
}

void Hit(uint32_t hit_from, int32_t damage) {
    Sys::Lock();
//    OutPulseOnHit.PulseI(settings.pulse_len_hit_ms);
    beeper.StartOrRestartI(bsqHit);
#if SIDE_LEDS_ENABLED
    for(auto& Led : side_LEDs) Led.StartOrRestartI(lsqHit);
#endif
    switch(state) {
        case IndiState::Idle:
            beeper.SetNextSequenceI(nullptr);
#if SIDE_LEDS_ENABLED
            side_LEDs[3].SetNextSequenceI(nullptr);
#endif
            break;
        case IndiState::HitsEnded:
            beeper.SetNextSequenceI(bsqHitsEnded);
#if SIDE_LEDS_ENABLED
            side_LEDs[3].SetNextSequenceI(lsqHitsEnded);
#endif
            break;
        case IndiState::Reloading:
            beeper.SetNextSequenceI(bsqReloading);
#if SIDE_LEDS_ENABLED
            side_LEDs[3].SetNextSequenceI(lsqReloading);
#endif
            break;
        case IndiState::MagazinesEnded:
            beeper.SetNextSequenceI(bsqMagazEnded);
#if SIDE_LEDS_ENABLED
            side_LEDs[3].SetNextSequenceI(lsqMagazinesEnded);
#endif
            break;
    }
    Sys::Unlock();
    Printf("#Hit from %d; damage %d; %d left\r\n", hit_from, damage, hit_cnt);
}

void HitsEnded() {
    Sys::Lock();
    // Beeper
    if(beeper.GetCurrentSequence() == bsqHit) beeper.SetNextSequenceI(bsqHitsEnded);
    else beeper.StartOrRestartI(bsqHitsEnded);
    output_hits_ended.SetHi();
#if SIDE_LEDS_ENABLED
    for(auto& Led : side_LEDs) Led.StartOrRestartI(lsqHitsEnded);
#endif
    Sys::Unlock();
    Printf("#Hits Ended\r\n");
}

void Reset(bool quiet) {
    state = IndiState::Idle;
    for(auto& Led : front_LEDs) Led.Stop();
#if SIDE_LEDS_ENABLED
    for(auto& Led : side_LEDs)  Led.Stop();
#endif
    if(!quiet) Printf("#Reset\r\n");
}

} // namespace
#endif

#if 1 // ========================= Reception processing ========================
static systime_t prev_hit_time_st = 0;

void IrRxCallbackI(uint32_t rcvd) { evt_q_app.SendNowOrExitI(AppMsg(AppEvt::IrRx, rcvd)); }

void ProcessRxPkt(IRPkt rx_pkt) {
//    rx_pkt.Print();
    // Reset
    if(rx_pkt.word16 == PKT_RESET) evt_q_app.SendNowOrExit(AppEvt::Reset);
    // Shot incoming
    else if(rx_pkt.zero == 0) {
        if(hit_cnt <= 0) return; // Nothing to do when no hits left
        // Ignore friendly fire (and pkt from self, too)
        if(rx_pkt.team_id == settings.team_id) return;
        // Ignore if not enough time passed since last hit
        if(Sys::TimeElapsedSince(prev_hit_time_st) < TIME_S2I(settings.min_delay_btw_hits)) return;
        // Hit occured, decrement if not infinity
        int32_t damage = rx_pkt.GetDamageHits();
        if(!settings.hit_cnt.IsInfinity()) {
            hit_cnt = (damage < hit_cnt)? (hit_cnt - damage) : 0;
        }
        Indication::Hit(rx_pkt.player_id, damage);
        if(hit_cnt > 0) prev_hit_time_st = Sys::GetSysTimeX();
        else Indication::HitsEnded();
    } // if zero
}
#endif

#if 1 // =============================== Firing ================================
VirtualTimer fire_tmr;
IRPkt pkt_tx;
systime_t fire_start_time_st;

// ==== Delay subsystem ====
void TmrCallback(void *p) {
    Sys::LockFromIRQ();
    evt_q_app.SendNowOrExitI((AppEvt)((uint32_t)p));
    Sys::UnlockFromIRQ();
}

void StartDelay(int32_t ADelay_s, AppEvt AEvt) {
    if(ADelay_s <= 0) evt_q_app.SendNowOrExit(AEvt); // Do it immediately
    else fire_tmr.Set(TIME_S2I(ADelay_s), TmrCallback, (void*)((uint32_t)AEvt));
}

void StartDelay_ms(int32_t ADelay_ms, AppEvt AEvt) {
    if(ADelay_ms <= 0) evt_q_app.SendNowOrExit(AEvt); // Do it immediately
    else fire_tmr.Set(TIME_MS2I(ADelay_ms), TmrCallback, (void*)((uint32_t)AEvt));
}

void OnIrTxEndI() { evt_q_app.SendNowOrExitI(AppEvt::EndOfIrTx); }

bool IsFiring;

void Fire() {
    IsFiring = true;
    if(!settings.rounds_in_magaz.IsInfinity()) rounds_cnt--;
    // Prepare pkt
    if(settings.pkt_type == PKT_SHOT) {
        pkt_tx.word16 = 0;
        pkt_tx.player_id = settings.player_id;
        pkt_tx.team_id  = settings.team_id;
        pkt_tx.damage_id = 0; // Which means 1 hit // XXX
        irLed::TransmitWord(pkt_tx.word16, 14, settings.ir_tx_pwr, OnIrTxEndI);
    }
    else {
        pkt_tx.word16 = (uint16_t)settings.pkt_type;
        irLed::TransmitWord(pkt_tx.word16, 16, settings.ir_tx_pwr, OnIrTxEndI);
    }
//    PktTx.Print();
    fire_start_time_st = Sys::GetSysTimeX();
    Indication::Shot();
}
#endif

void Reset(bool quiet) {
    Sys::Lock();
    irLed::ResetI();
    output_hits_ended.ResetI();
//    OutPulseOnHit.ResetI();  // Removed to implement PWM input
    input_burst_fire.CleanIrqFlag();
    input_single_fire.CleanIrqFlag();
    IsFiring = false;
    rounds_cnt = settings.rounds_in_magaz;
    magazines_cnt = settings.magazines_cnt;
    fire_tmr.ResetI();
    hit_cnt = settings.hit_cnt;
    prev_hit_time_st = Sys::GetSysTimeX();
    // IR tx
    irLed::SetCarrierFreq(settings.ir_tx_freq);
    Sys::Unlock();
    Indication::Reset(quiet);
}

// ==================================== Thread =================================
static THD_WORKSPACE(waAppThread, 256);
static void AppThread() {
    while(true) {
        AppMsg Msg = evt_q_app.Fetch(TIME_INFINITE);
        if(is_testing) { // Sleep if testing
            Sys::SleepMilliseconds(999);
            continue;
        }
        // Will be here when new Evt occur and if not in testing mode
        if(Msg.evt == AppEvt::Reset) Reset();
        else if(Msg.evt == AppEvt::IrRx) ProcessRxPkt(IRPkt(Msg.data16));
        else if(hit_cnt > 0) switch(Msg.evt) { // Do nothing if no hits left
            case AppEvt::StartFire:
                if(!IsFiring and rounds_cnt > 0) Fire();
                break;

            case AppEvt::NewPwmData:
//                Printf("pwm %u\r", input_pwm.pwm_duty);
                if(input_pwm.pwm_duty > PWM_DUTY_SINGLE_FIRE_percent and !IsFiring and rounds_cnt > 0) Fire();
                break;

            case AppEvt::EndOfIrTx: // Tx of several same pkts just ended
                StartDelay_ms(settings.shots_period_ms - TIME_I2MS(Sys::TimeElapsedSince(fire_start_time_st)), AppEvt::EndOfDelayBetweenShots);
                break;

            case AppEvt::EndOfDelayBetweenShots:
                IsFiring = false;
                if(rounds_cnt > 0) { // Fire if needed and there are rounds left
                    if(DoBurstFire()) Fire();
                }
                else { // no more rounds
                    if(magazines_cnt > 1) { // Reload if possible (more than 0 magazines left)
                        if(!settings.magazines_cnt.IsInfinity()) magazines_cnt--;
                        Indication::RoundsEnded();
                        StartDelay(settings.magaz_reload_delay, AppEvt::MagazineReloadDone);
                    }
                    else {
                        magazines_cnt = 0; // To avoid situation with 1 magaz with 0 rounds
                        Indication::MagazinesEnded(); // No magazines left
                    }
                }
                break;

            case AppEvt::MagazineReloadDone:
                Indication::MagazineReloaded();
                rounds_cnt = settings.rounds_in_magaz;
                if(DoBurstFire()) Fire();
                break;

            default: break;
        } // switch Evt
    } // while true
}

void AppInit() {
    evt_q_app.Init();
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);
    // Control pins init
    output_hits_ended.Init();
//    OutPulseOnHit.Init(); // Removed to implement PWM input
    input_burst_fire.Init();
    input_single_fire.Init();
    input_pwm.Init();
    Reset();
    // Create and start thread
    Sys::CreateThd(waAppThread, sizeof(waAppThread), NORMALPRIO, AppThread);
}
