/*
 * app_classes.h
 *
 *  Created on: 1 сент. 2024 г.
 *      Author: layst
 */

#ifndef APP_CLASSES_H_
#define APP_CLASSES_H_

#include "yartos.h"
#include "gd_lib.h"
#include "MsgQ.h"

#if 1 // ========================== Message queue ==============================
enum class AppEvt : uint8_t {
    Reset,
    StartFire, EndOfIrTx, EndOfDelayBetweenShots, MagazineReloadDone,
    IrRx,
    NewPwmData,
};

union AppMsg {
    uint32_t dword;
    struct {
        uint16_t data16;
        uint8_t data8;
        AppEvt evt;
    } __attribute__((__packed__));

    AppMsg& operator = (const AppMsg &Right) {
        dword = Right.dword;
        return *this;
    }
    AppMsg() : dword(0) {}
    AppMsg(AppEvt aevt) : data16(0), data8(0), evt(aevt) {}
    AppMsg(AppEvt aevt, uint16_t adata16) : data16(adata16), evt(aevt) {}
    AppMsg(AppEvt aevt, uint8_t adata8, uint16_t adata16) : data16(adata16), data8(adata8), evt(aevt) {}
} __attribute__((__packed__));

using AppMsgQueue = EvtMsgQ<AppMsg, 9>;

extern AppMsgQueue evt_q_app;
#endif

#if 1 // ============================= Pulser pin ==============================
void PulserCallback(void *p);

class PulserPin: private Pin_t {
private:
    VirtualTimer itmr;
    friend void PulserCallback(void *p);
    void IOnTmrDone() { SetLo(); }
public:
    PulserPin(GPIO_TypeDef *APGPIO, uint16_t APin) : Pin_t(APGPIO, APin) {}
    void Init() { Pin_t::SetupOut(gpio::PushPull); }
    void PulseI(uint32_t Dur);
    void ResetI();
    void SetHi() { Pin_t::SetHi(); }
    void SetLo() { Pin_t::SetLo(); }
};
#endif

#if 1 // ============================ AppOutPin ================================

#endif

#if 1 // ======================== Input pin: IRQ & Timed =======================
void InputPinIrqHandlerI();

class InputPinIrqTimed : public PinIrq {
private:
    systime_t start_time_st = 0;
public:
    InputPinIrqTimed(GPIO_TypeDef *apgpio, uint16_t apin_n, gpio::PullUpDown apull_up_down):
        PinIrq(apgpio, apin_n, apull_up_down, InputPinIrqHandlerI) {}
    void Init() {
        PinIrq::Init(ttRising);
        CleanIrqFlag();
        EnableIrq(IRQ_PRIO_LOW);
    }
    bool CheckIfProcess() {
        if(Sys::TimeElapsedSince(start_time_st) < TIME_MS2I(INPUT_DEADTTIME_ms)) return false; // Not enough time passed
        else {
            start_time_st = Sys::GetSysTimeX();
            return true;
        }
    }
};
#endif

#if 1 // =========================== PWM input =================================
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
    gpio::PullUpDown pull_up_down;
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

    PwmInputPin(GPIO_TypeDef *apgpio, uint16_t apin_n, gpio::PullUpDown apull_up_down,
            TIM_TypeDef *aptimer) : HwTim(aptimer),
                pgpio(apgpio), pin_n(apin_n), pull_up_down(apull_up_down) {}

    void Init() {
        gpio::SetupInput(pgpio, pin_n, pull_up_down);
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
#endif

#endif /* APP_CLASSES_H_ */
