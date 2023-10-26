#include "gd_lib.h"
#include "gd_uart.h"
#include "yartos.h"
#include "MsgQ.h"
#include "usb_cdc.h"
#include "usb.h"
#include "SpiFlash.h"

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

SpiFlash_t SpiFlash(SPI0);

EvtTimer_t TmrUartCheck(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer_t::Type::Periodic);

//PinOutputPWM_t LedR{ LED_R };
//PinOutputPWM_t LedG{ LED_G };
//PinOutputPWM_t LedB{ LED_B };

#endif

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

    Gpio::SetupOut(PA10, Gpio::PushPull);
    Gpio::SetHi(PA10);

    RCU->EnAFIO();
    // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST
    AFIO->DisableJtagDP();

    Uart.Init();
    // RTOS & Event queue
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk::PrintFreqs();

    AFIO->RemapSPI0_PB345();
    SpiFlash.Init(FLASH_NSS, FLASH_SCK, FLASH_MISO, FLASH_MOSI, FLASH_IO2, FLASH_IO3);
    SpiFlash.Reset();
    Printf("FlashID: %X\r", SpiFlash.ReleasePowerDown());


//    if(!XtalIsOk) Printf("XTAL Fail\r");
    TmrUartCheck.StartOrRestart();



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
            SpiFlash.Read(Addr, Buf, Len);
            Printf("%A\r", Buf, Len, ' ');
        }
        else PShell->BadParam();
    }

    /*
    else if(PCmd->NameIs("ShaFw")) {
        uint32_t Sz;
//        if(PCmd->GetNext(&Sz) != retvOk) Sz = ZMODULE_FW_SZ;
        uint8_t hash[32];
        const char* key = "1234567890";
        DBG_HI();
//        Sha256DoHash((uint8_t*)ZModuleFw, Sz, hash);
        Sha256DoHash((uint8_t*)key, strlen(key), hash);
        DBG_LO();
        Printf("%A\r", hash, 32, ' ');
    }

    // ==== Crypto ====
    else if(PCmd->NameIs("GetSn")) { Printf("%S\r", CraCry.SNStr); }
    else if(PCmd->NameIs("SignIt")) {
        char *S = PCmd->GetNextString();
        uint32_t Sz = strlen(S);
        if(Sz) {
            CraCry.SignIt(S, Sz);
            Printf("%A\r", CraCry.Digest, SHA256HashSize, ' ');
        }
        else PShell->BadParam();
    }
*/

//    else if(PCmd->NameIs("HMAC")) {
//        uint8_t hash[32];
//        uint32_t KeySz, MsgSz;
//        if(PCmd->GetNext(&KeySz) == retvOk and PCmd->GetNext(&MsgSz) == retvOk) {
//            uint8_t *pKey = (uint8_t*)ZModuleFw;
//            uint8_t *pMsg = pKey + MsgSz;
//            DBG_HI();
//            HMAC_SHA256(pMsg, MsgSz, pKey, KeySz, hash);
//            DBG_LO();
//            Printf("Key %u, Msg %u: %A\r", KeySz, MsgSz, hash, 32, ' ');
//        }
//        else {
//            const char* txt = "The quick brown fox jumps over the lazy dog";
//            const char* key = "key";
//            HMAC_SHA256((uint8_t*)txt, strlen(txt), (uint8_t*)key, strlen(key), hash);
//            Printf("%A\r", hash, 32, ' ');
//        }
//    }

    else PShell->CmdUnknown();
}
