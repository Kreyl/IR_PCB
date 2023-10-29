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

#if 1 // ======================== Variables and defines ========================
// Debug pin
//#define DBG_PIN         PA3
//#define DBG_HI()        PinSetHi(DBG_PIN)
//#define DBG_LO()        PinSetLo(DBG_PIN)
//#define DBG_TOGGLE()    PinToggle(DBG_PIN)

// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);

LedSmooth_t Lumos{LUMOS_PIN};
LedSmooth_t SideLEDs[SIDE_LEDS_CNT] = { {LED_PWM1}, {LED_PWM2}, {LED_PWM3}, {LED_PWM4} };
LedSmooth_t FrontLEDs[FRONT_LEDS_CNT] = { {LED_FRONT1}, {LED_FRONT2} };

static const NpxParams NParams{NPX_PARAMS, NPX_DMA, 4, NpxParams::ClrType::RGB};
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

    // LEDs
    for(auto &Led : SideLEDs) {
        Led.Init();
        Led.StartOrRestart(lsqFadeInOut);
        Sys::SleepMilliseconds(207);
    }
    for(auto &Led : FrontLEDs) {
        Led.Init();
        Led.StartOrRestart(lsqFadeInOut);
        Sys::SleepMilliseconds(207);
    }
//    Lumos.Init();
//    Lumos.StartOrRestart(lsqFadeIn);
    Gpio::SetupOut(PA10, Gpio::PushPull);
    Gpio::SetHi(PA10); // XXX

    NpxLeds.Init();

    AFIO->RemapSPI0_PB345();
    SpiFlash.Init();
    SpiFlash.Reset();
//    Printf("FlashID: %X\r", SpiFlash.ReleasePowerDown());

    TmrUartCheck.StartOrRestart();

    irLed::Init();
    irRcvr::Init(IrRxCallbackI);

    // DEBUG
//    Gpio::SetupOut(PB0, Gpio::PushPull);


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

uint8_t buf[128];

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

    else if(PCmd->NameIs("bin")) {
        uint32_t Sz, Delay;
        if(PCmd->GetNext(&Sz) == retv::Ok and Sz > 0 and PCmd->GetNext(&Delay) == retv::Ok) {
            if(UsbCDC.ReceiveBinaryToBuf(buf, Sz, Delay) == retv::Ok) {
                PShell->Print("%A\r", buf, Sz, ' ');
                if(UsbCDC.TransmitBinaryFromBuf(buf, Sz, Delay) == retv::Ok) PShell->Ok();
                else PShell->Failure();
            }
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("trim")) {
        Printf("Stat: 0x%X\r", (CTC->STAT & 0xFF));
        Printf("trimval: %u\r", (CTC->CTL0 >> 8));
        Printf("refcap: %u\r", (CTC->STAT >> 16));
        CTC->INTC = 0x0FUL;
    }

    else if(PCmd->NameIs("Led")) {
        uint32_t R, G, B;
        if(PCmd->GetParams<uint32_t>(3, &R, &G, &B) == retv::Ok) {
//            LedR.Set(R);
//            LedG.Set(G);
//            LedB.Set(B);
            PShell->Ok();
            return;
        }
        PShell->BadParam();
    }

    else if(PCmd->NameIs("FSta")) {
        Printf("0x%02X 0x%02X 0x%02X\r",
                SpiFlash.ReadStatusReg1(),
                SpiFlash.ReadStatusReg2(),
                SpiFlash.ReadStatusReg3());
    }

    else if(PCmd->NameIs("FMfr")) {
        SpiFlash_t::MfrDevId_t r;
        r = SpiFlash.ReadMfrDevId();
        Printf("0x%02X 0x%02X\r", r.Mfr, r.DevID);
    }
    else if(PCmd->NameIs("FMfrQ")) {
        SpiFlash_t::MfrDevId_t r;
        r = SpiFlash.ReadMfrDevIdQ();
        Printf("0x%02X 0x%02X\r", r.Mfr, r.DevID);
    }

    else if(PCmd->NameIs("FR")) {
        uint32_t Addr, Len;
        if(PCmd->GetParams<uint32_t>(2, &Addr, &Len) == retv::Ok) {
            uint8_t Buf[Len];
            if(SpiFlash.Read(Addr, Buf, Len) == retv::Ok) Printf("%A\r", Buf, Len, ' ');
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("FRQ")) {
        uint32_t Addr, Len;
        if(PCmd->GetParams<uint32_t>(2, &Addr, &Len) == retv::Ok) {
            uint8_t Buf[Len];
            if(SpiFlash.ReadQ(Addr, Buf, Len) == retv::Ok) Printf("%A\r", Buf, Len, ' ');
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("FE")) {
        uint32_t Addr;
        if(PCmd->GetNext(&Addr) == retv::Ok) {
            if(SpiFlash.EraseSector4k(Addr) == retv::Ok) PShell->Ok();
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("FW")) {
        uint32_t Addr;
        if(PCmd->GetNext(&Addr) == retv::Ok) {
            uint8_t Buf[16];
            for(uint32_t i=0; i<16; i++) Buf[i] = i;
            if(SpiFlash.WritePageQ(Addr, Buf, 16) == retv::Ok) PShell->Ok();
            else PShell->Failure();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("IRTX")) {
        uint16_t w, Pwr;
        if(PCmd->GetParams<uint16_t>(2, &w, &Pwr) == retv::Ok) {
            irLed::TransmitWord(w, 16, Pwr, nullptr);
            PShell->Ok();
        }
        else PShell->BadParam();
    }


    else PShell->CmdUnknown();
}
