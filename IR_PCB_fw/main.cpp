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

Settings_t Settings;

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

    /*
    Beeper.Init();
    Beeper.StartOrRestart(bsqWeAreTheChampions);

    // LEDs
    for(auto &Led : SideLEDs) {
        Led.Init();
        Led.StartOrRestart(lsqFadeInOut);
        chThdSleepMilliseconds(207);
    }
    for(auto &Led : FrontLEDs) {
        Led.Init();
        Led.StartOrRestart(lsqFadeInOut);
        chThdSleepMilliseconds(207);
    }

    Lumos.Init();
    Lumos.StartOrRestart(lsqFadeIn);
*/

    AppInit();
    irLed.Init();
//    DAC->CR = DAC_CR_EN1;

//    Settings.Load();

//    SimpleSensors::Init();



//    PinSetupOut(GPIOA, 0, omPushPull);
//    PinSetHi(GPIOA, 0);

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


uint8_t Crc3Rem[256] = {
        0x00,0x60,0xC0,0xA0,0xE0,0x80,0x20,0x40,0xA0,0xC0,0x60,0x00,0x40,0x20,0x80,0xE0,
        0x20,0x40,0xE0,0x80,0xC0,0xA0,0x00,0x60,0x80,0xE0,0x40,0x20,0x60,0x00,0xA0,0xC0,
        0x40,0x20,0x80,0xE0,0xA0,0xC0,0x60,0x00,0xE0,0x80,0x20,0x40,0x00,0x60,0xC0,0xA0,
        0x60,0x00,0xA0,0xC0,0x80,0xE0,0x40,0x20,0xC0,0xA0,0x00,0x60,0x20,0x40,0xE0,0x80,
        0x80,0xE0,0x40,0x20,0x60,0x00,0xA0,0xC0,0x20,0x40,0xE0,0x80,0xC0,0xA0,0x00,0x60,
        0xA0,0xC0,0x60,0x00,0x40,0x20,0x80,0xE0,0x00,0x60,0xC0,0xA0,0xE0,0x80,0x20,0x40,
        0xC0,0xA0,0x00,0x60,0x20,0x40,0xE0,0x80,0x60,0x00,0xA0,0xC0,0x80,0xE0,0x40,0x20,
        0xE0,0x80,0x20,0x40,0x00,0x60,0xC0,0xA0,0x40,0x20,0x80,0xE0,0xA0,0xC0,0x60,0x00,
        0x60,0x00,0xA0,0xC0,0x80,0xE0,0x40,0x20,0xC0,0xA0,0x00,0x60,0x20,0x40,0xE0,0x80,
        0x40,0x20,0x80,0xE0,0xA0,0xC0,0x60,0x00,0xE0,0x80,0x20,0x40,0x00,0x60,0xC0,0xA0,
        0x20,0x40,0xE0,0x80,0xC0,0xA0,0x00,0x60,0x80,0xE0,0x40,0x20,0x60,0x00,0xA0,0xC0,
        0x00,0x60,0xC0,0xA0,0xE0,0x80,0x20,0x40,0xA0,0xC0,0x60,0x00,0x40,0x20,0x80,0xE0,
        0xE0,0x80,0x20,0x40,0x00,0x60,0xC0,0xA0,0x40,0x20,0x80,0xE0,0xA0,0xC0,0x60,0x00,
        0xC0,0xA0,0x00,0x60,0x20,0x40,0xE0,0x80,0x60,0x00,0xA0,0xC0,0x80,0xE0,0x40,0x20,
        0xA0,0xC0,0x60,0x00,0x40,0x20,0x80,0xE0,0x00,0x60,0xC0,0xA0,0xE0,0x80,0x20,0x40,
        0x80,0xE0,0x40,0x20,0x60,0x00,0xA0,0xC0,0x20,0x40,0xE0,0x80,0xC0,0xA0,0x00,0x60,
};

#if 1 // ======================= Command processing ============================
void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
//    Printf("%S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("Version: %S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
//    else if(PCmd->NameIs("mem")) PrintMemoryInfo();

    else if(PCmd->NameIs("CtrlSet")) {
        uint32_t In[3];
        for(int i=0; i<3; i++) {
            if(PCmd->GetNext(&In[i]) != retvOk) break;
            SetInputs(In);
        }
        PShell->Ok();
    }


    else if(PCmd->NameIs("crc")) {
        uint32_t N;
        if(PCmd->GetNext(&N) != retvOk) { PShell->BadParam(); return; }

        uint32_t Rslts[8] = { 0,0,0,0,0,0,0,0 };

        uint32_t Poly = 0xB0000000;

        systime_t Start = chVTGetSystemTimeX();
//        for(uint32_t i=0; i<N; i++) {
            for(uint32_t v = 0; v<=0x1FFF; v++) {
                uint32_t rem = v << 19;

                uint32_t top = (rem >> 24) & 0xFFUL;
                rem = ((uint32_t)Crc3Rem[top] << 24) | (rem & 0x00FF0000UL);

                for(uint32_t j=0; j<13; j++) {
                    if(rem & 0x80000000) rem ^= Poly;
                    rem <<= 1;
                }


//                for(uint32_t j=0; j<13; j++) {
//                    if(rem & 0x80000000) rem ^= Poly;
//                    rem <<= 1;
//                }
                uint32_t crc = (rem >> 29) & 0b111UL;
                if(crc > 7) Printf("err 0x%X\r", v);
                else Rslts[crc]++;
                Printf("%u crc: 0x%X\r", v, crc);
            }
//        }
        sysinterval_t Dur = chVTTimeElapsedSinceX(Start);
        for(uint32_t i=0; i<8; i++) Printf("%u: %u\r", i, Rslts[i]);
        Printf("Dur: %u\r", TIME_I2MS(Dur));
//        Printf("crc: 0x%X\r", crc);
//        Printf("Done\r");
    }

    else if(PCmd->NameIs("tbl")) {
        uint32_t Poly = 0xB0000000;

        for(uint32_t v = 0; v < 256; v++) {
            uint32_t rem = v << 24;
            for(uint32_t j=0; j < 8; j++) {
                if(rem & 0x80000000) rem ^= Poly;
                rem <<= 1;
            }
            Printf("0x%02X,", (rem >> 24));
            if((v+1) % 16 == 0) Printf("\r\n");
        }
    }

    else if(PCmd->NameIs("irtx")) {
        uint32_t N, Pwr;
        if(PCmd->GetNext(&N) != retvOk) { PShell->BadParam(); return; }
        if(PCmd->GetNext(&Pwr) != retvOk) { PShell->BadParam(); return; }
        irLed.TransmitWord(N, Pwr);
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
