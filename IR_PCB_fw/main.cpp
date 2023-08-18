#include "ch.h"
#include "hal.h"
#include "MsgQ.h"
#include "shell.h"
#include "uart.h"
#include "kl_lib.h"
#include "SimpleSensors.h"
#include "buttons.h"
#include "Sequences.h"
#include "beeper.h"
#include "adcF072.h"
#include "Settings.h"
#include "battery_consts.h"
#include "led.h"
#include "FwUpdateF072.h"
#include "app.h"
#include "ir.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);
void ITask();

LedSmooth_t Lumos{LUMOS_PIN};
LedSmooth_t SideLEDs[SIDE_LEDS_CNT] = { {LED_PWM1}, {LED_PWM2}, {LED_PWM3}, {LED_PWM4} };
LedSmooth_t FrontLEDs[FRONT_LEDS_CNT] = { {LED_FRONT1}, {LED_FRONT2} };

Beeper_t Beeper {BEEPER_PIN};

// === Vector table moving to SRAM ==
#if FROM_BOOT
static inline void MoveVectorTable() {
    __disable_irq();
    // Copy table
    volatile uint32_t *VTable = (volatile uint32_t*)0x20000000; // Start of RAM
    for(uint32_t i=0; i<48; i++) VTable[i] = *(volatile uint32_t*)((uint32_t)FW_START_ADDR + (i << 2));
    // Reset all
    rccResetAHB(0x017E0000);
    rccResetAPB1(0x7AEE4933);
    rccResetAPB2(0x00475A01);
    // En Syscfg clk and do remap
    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, 0);
    SYSCFG->CFGR1 = 0b11UL; // Embedded SRAM mapped at 0x00000000
}
#endif
#endif




void main() {
#if FROM_BOOT
//    MoveVectorTable();
#endif
    // ==== Init Clock system ====
    Clk.EnablePrefetch();
    Clk.SetupFlashLatency(20);
    bool QuartzIsOk = false;
    if(Clk.EnableHSE() == retvOk) {
        Clk.SetupPLLSrc(plsHSEdivPREDIV); // 4MHz
        if(Clk.SetupPLLDividers(3, pllMul5) == retvOk) { // 12 / 3 * 5 = 20MHz
            Clk.SetupBusDividers(ahbDiv1, apbDiv1);
            QuartzIsOk = Clk.SwitchTo(csPLL) == retvOk;
        }
    }
    Clk.UpdateFreqValues();

    // === Init OS ===
    halInit();
    chSysInit();

    // ==== Init hardware ====
    EvtQMain.Init();
    Uart.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    if(!QuartzIsOk) Printf("Quartz fail\r\n");
    Clk.PrintFreqs();

    Beeper.Init();
//    Beeper.StartOrRestart(bsqWeAreTheChampions);
    Settings.Load();

    // LEDs
    for(auto &Led : SideLEDs) {
        Led.Init();
//        Led.StartOrRestart(lsqFadeInOut);
//        chThdSleepMilliseconds(207);
    }
    for(auto &Led : FrontLEDs) {
        Led.Init();
//        Led.StartOrRestart(lsqFadeInOut);
//        chThdSleepMilliseconds(207);
    }

    Lumos.Init();
    Lumos.StartOrRestart(lsqFadeIn);
//    while(!FrontLEDs[1].IsIdle()) chThdSleepMilliseconds(45);

//    SimpleSensors::Init();
    AppInit();

    ITask(); // Main cycle
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdShellCmdRcvd:
                while(((CmdUart_t*)Msg.Ptr)->TryParseRxBuff() == retvOk) OnCmd((Shell_t*)((CmdUart_t*)Msg.Ptr));
                break;

            default: break;
        } // switch
    } // while true
}

void ProcessInput(PinSnsState_t *PState, uint32_t Len) {

}

#if 1 // ======================= Command processing ============================
void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
//    Printf("%S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("Version: %S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
//    else if(PCmd->NameIs("mem")) PrintMemoryInfo();

    else if(PCmd->NameIs("GetSta")) {
        Printf("Hits: %d; Rnds: %d; mgzs: %d\r", HitCnt, RoundsCnt, MagazinesCnt);
    }
    else if(PCmd->NameIs("Restore")) {
        Reset();
        PShell->Ok();
    }

    // ==== Settings ====
    else if(PCmd->NameIs("GetSettings")) {
        Value_t *Arr = (Value_t*)&Settings;
        for(uint32_t i=0; i<SETTINGS_CNT; i++, Arr++) {
            Printf("%*S = %4u; Min = %u; Max = %4u; default = %4u\r\n",
                    16, Arr->Name, Arr->v, Arr->Min, Arr->Max, Arr->Default);
        }
    }

    else if(PCmd->NameIs("Set")) {
        const char* Name;
        uint32_t v, N = 0;
        bool Found;
        // Get pairs of values
        while((((Name = PCmd->GetNextString()) != nullptr) and PCmd->GetNext(&v) == retvOk)) {
            Found = false;
            Value_t *Arr = (Value_t*)&Settings;
            for(uint32_t i=0; i<SETTINGS_CNT; i++, Arr++) { // Find by name
                if(kl_strcasecmp(Name, Arr->Name) == 0) {
                    if(Arr->CheckAndSetIfOk(v)) {
                        Printf("%S = %u\r\n", Name, v);
                        N++;
                        Found = true;
                        break;
                    }
                    else {
                        Printf("%S BadValue: %u\r\n", Name, v);
                        return;
                    }
                } // if
            } // for
            if(!Found) {
                Printf("BadName: %S\r\n", Name);
                break;
            }
        } // while
        Printf("Set %u values\r\n", N);
        Reset();
    }

    else if(PCmd->NameIs("SaveSettings")) {
        if(Settings.Save() == retvOk) Printf("Saved\r\n");
        else Printf("Saving fail\r\n");
    }

    else if(PCmd->NameIs("LoadSettings")) {
        Settings.Load();
        Reset();
    }

    // ==== Debug ====
    else if(PCmd->NameIs("CtrlSet")) {
        uint32_t In[3] = { 0, 0, 0 };
        for(int i=0; i<3; i++) {
            if(PCmd->GetNext(&In[i]) != retvOk) break;
        }
        SetInputs(In);
        PShell->Ok();
    }
    else if(PCmd->NameIs("irtx")) {
        uint32_t Word, Pwr;
        if(PCmd->GetNext(&Word) != retvOk) { PShell->BadParam(); return; }
        if(PCmd->GetNext(&Pwr) != retvOk) { PShell->BadParam(); return; }
        irLed::TransmitWord(Word, Pwr, nullptr);
        PShell->Ok();
    }

    else if(PCmd->NameIs("dac")) {
        uint8_t N;
        if(PCmd->GetNext(&N) != retvOk) { PShell->BadParam(); return; }
        DAC->DHR8R1 = N;
        Printf("%u\r", DAC->DHR8R1);
        PShell->Ok();
    }

#if 0 // ==== FW Update ====
    else if(PCmd->NameIs("UpdateFwRestart")) {
        Flames.StopNow();
        FwUpdater.Restart();
        PShell->Ok();
    }

    else if(PCmd->NameIs("UpdateFwWrite")) {
        Iwdg::Reload();
        int32_t Sz;
        if(PCmd->GetNext<int32_t>(&Sz) != retvOk or Sz < 1 or Sz > FW_UPD_BUF_SZ) { PShell->BadParam(); return; }
        uint32_t crc32;
        if(PCmd->GetNext<uint32_t>(&crc32) != retvOk) { PShell->BadParam(); return; }
        if(PShell->ReceiveBinaryToBuf(FwUpdater.Buf, Sz, 4500) == retvOk) {
            Iwdg::Reload();
            uint16_t crc16 = crc32;
            switch(FwUpdater.WriteBuf(Sz, &crc16)) {
                case retvOk:       PShell->Ok(); break;
                case retvTimeout:  PShell->Timeout(); break;
                case retvCRCError: PShell->CRCError(); break;
                default: PShell->Failure(); break;
            }
        }
        else PShell->Failure();
    }
    else if(PCmd->NameIs("UpdateFwCheckAndRun")) {
        Iwdg::Reload();
        uint32_t crc32;
        if(PCmd->GetNext<uint32_t>(&crc32) != retvOk) { PShell->BadParam(); return; }
        uint16_t crc16 = crc32;
        switch(FwUpdater.CheckItAll(&crc16)) {
            case retvOk:
                PShell->Ok();
                chThdSleepMilliseconds(45);
                FwUpdater.JumpToBootloader();
                break;
            case retvCRCError: PShell->CRCError(); break;
            default: PShell->Failure(); break;
        }
    }
#endif

    else PShell->CmdUnknown();
}
#endif
