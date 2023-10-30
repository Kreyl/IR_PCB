#include "gd_lib.h"
#include "gd_uart.h"
#include "yartos.h"
#include "MsgQ.h"
#include "usb_cdc.h"
#include "usb.h"
#include "SpiFlash.h"
#include "led.h"
#include "beeper.h"
#include "Sequences.h"
#include "ir.h"
#include "ws2812bTim.h"
#include "max98357.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);

LedSmooth_t Lumos{LUMOS_PIN};
LedSmooth_t SideLEDs[SIDE_LEDS_CNT] = { {LED_PWM1}, {LED_PWM2}, {LED_PWM3}, {LED_PWM4} };
LedSmooth_t FrontLEDs[FRONT_LEDS_CNT] = { {LED_FRONT1}, {LED_FRONT2} };

static const NpxParams NParams{NPX_PARAMS, NPX_DMA, 17, NpxParams::ClrType::RGB};
Neopixels_t NpxLeds{&NParams};

Beeper_t Beeper {BEEPER_PIN};

SpiFlash_t SpiFlash(SPI0);

EvtTimer_t TmrUartCheck(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer_t::Type::Periodic);

#endif

void IrRxCallbackI(uint32_t Rcvd) { PrintfI("RX: 0x%X\r", Rcvd); }

void main(void) {
    FMC->EnableCashAndPrefetch();
    RCU->EnPwrMgmtUnit();

    // ==== Setup clock ====
    FMC->SetLatency(50);
    if(RCU->EnableXTAL() == retv::Ok) {
//        RCU->SetCK48MSel_CKPLL(); // USB clock src is PLL
        // Setup system clock
        RCU->SetPllPresel_XTAL();
        RCU->SetPrediv0Sel_XTALorIRC48M(); // XTAL is a source
        // 12MHz div 6 = 2MHz; 2MHz *25 = 50MHz. PLL input freq must be in [1; 25] MHz, 8MHz typical
        RCU->SetPrediv0(6);
        RCU->SetPllSel_Prediv0();
        RCU->SetPllMulti(PllMulti::mul25);
        // Switch clk
        if(RCU->EnablePll() == retv::Ok) {
            RCU->SetAhbPrescaler(AhbPsc::div1);
            RCU->SwitchCkSys2PLL();
        }
    }
    /*
    FMC->SetLatency(48);
    if(RCU->EnableXTAL() == retv::Ok) {
        RCU->SetCK48MSel_CKPLL(); // USB clock src is PLL
        // Setup system clock
        RCU->SetPllPresel_XTAL();
        RCU->SetPrediv0Sel_XTALorIRC48M(); // XTAL is a source
        // 12MHz div 1 = 12MHz; 12MHz *4 = 48MHz. PLL input freq must be in [1; 25] MHz, 8MHz typical
        RCU->SetPrediv0(1);
        RCU->SetPllSel_Prediv0();
        RCU->SetPllMulti(PllMulti::mul04);
        // Switch clk
        if(RCU->EnablePll() == retv::Ok) {
            RCU->SetAhbPrescaler(AhbPsc::div1);
            RCU->SwitchCkSys2PLL();
        }
    }
    */
    Clk::UpdateFreqValues();

    RCU->EnAFIO();
    // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST
    AFIO->DisableJtagDP();

    Uart.Init();
    // RTOS & Event queue
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk::PrintFreqs();
    //    if(!XtalIsOk) Printf("XTAL Fail\r");

    Beeper.Init();
//    Beeper.StartOrRestart(bsqShot);

//    Gpio::SetupOut(AU_SDMODE, Gpio::PushPull);
//    Gpio::SetHi(AU_SDMODE);

    // LEDs
    for(auto &Led : SideLEDs) {
        Led.Init();
//        Led.StartOrRestart(lsqFadeInOut);
//        Sys::SleepMilliseconds(207);
    }
    for(auto &Led : FrontLEDs) {
        Led.Init();
//        Led.StartOrRestart(lsqFadeInOut);
//        Sys::SleepMilliseconds(207);
    }
//    Lumos.Init();
//    Lumos.StartOrRestart(lsqFadeIn);
    Gpio::SetupOut(PA10, Gpio::PushPull);
    Gpio::SetHi(PA10); // XXX

    NpxLeds.Init();
    NpxLeds.SetAll(clGreen);
    NpxLeds.SetCurrentColors();

    AFIO->RemapSPI0_PB345();
    SpiFlash.Init();
    SpiFlash.Reset();
    Printf("FlashID: %X\r", SpiFlash.ReleasePowerDown());

//    Codec::Init();

//    irLed::Init();
//    irRcvr::Init(IrRxCallbackI);

    TmrUartCheck.StartOrRestart();

    // Main evt cycle
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case EvtId::UartCheckTime:
                Gpio::Toggle(PA10);
                while(Uart.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&Uart);
                break;

            case EvtId::UsbDataRcvd:
                while(UsbCDC.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&UsbCDC);
                break;

            case EvtId::EverySecond:
                Printf("aga %u\r", Msg.Value);
                break;

            case EvtId::UsbReady:
                Printf("Usb ready\r");
                CTC->Enable(); // Start autotrimming of IRC48M
                break;

            default: break;
        } // switch
    }
}

//uint16_t buf[16];
static const int32_t sinbuf[] = {
0, 1389, 2736, 3999, 5142, 6128, 6928, 7517, 7878, 8000,
7878, 7517, 6928, 6128, 5142, 4000, 2736, 1389, 0, -1389,
-2736, -4000, -5142, -6128, -6928, -7517, -7878, -8000, -7878, -7517,
-6928, -6128, -5142, -3999, -2736, -1389};
#define SIN_SZ    36

bool Proceed = false;

void I2SDmaDoneCbI() {
    if(Proceed) {
        Codec::TransmitBuf((void*)sinbuf, SIN_SZ);
    }
//    PrintfI("aga\r");
}

void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else if(PCmd->NameIs("conn")) {
        UsbCDC.Connect();
        PShell->Ok();
    }
    else if(PCmd->NameIs("dsc")) {
        UsbCDC.Disconnect();
        PShell->Ok();
    }

    else if(PCmd->NameIs("IRTX")) {
        uint16_t w, Pwr;
        if(PCmd->GetParams<uint16_t>(2, &w, &Pwr) == retv::Ok) {
            irLed::TransmitWord(w, 16, Pwr, nullptr);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("I2STX")) {
        Proceed = false;
        uint32_t w;
        if(PCmd->GetNext(&w) == retv::Ok) {
            if(Codec::SetupSampleRate(w) == retv::Ok) {
                Codec::I2SDmaDoneCbI = I2SDmaDoneCbI;
                Proceed = true;
                Codec::TransmitBuf((void*)sinbuf, SIN_SZ);
                PShell->Ok();
            }
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("stop")) {
        Proceed = false;
    }

    else if(PCmd->NameIs("Npx")) {
        Color_t clr;
        if(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.SetAll(clr);
//        uint32_t i=0;
//        while(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.ClrBuf[i++] = clr;
        NpxLeds.SetCurrentColors();
    }

    else PShell->CmdUnknown();
}
