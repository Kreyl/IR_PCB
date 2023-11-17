#include "gd_lib.h"
#include "gd_uart.h"
#include "yartos.h"
#include "MsgQ.h"
#include "usb_msdcdc.h"
#include "usb.h"
#include "SpiFlash.h"
#include "led.h"
#include "Sequences.h"
#include "mem_msd_glue.h"
#include "kl_fs_utils.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);

LedSmooth_t Lumos{LUMOS_PIN};
SpiFlash_t SpiFlash(SPI0);
FATFS FlashFS;
FIL sFile;

//uint32_t Buf[(PAGE_SZ / sizeof(uint32_t))];


EvtTimer_t TmrUartCheck(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer_t::Type::Periodic);
#endif

void JumpToApp();

void ErasePage(uint32_t PageAddr);
void WriteToMemory(uint32_t *PAddr, uint32_t *PBuf, uint32_t ALen);

static inline bool AppIsEmpty() {
    uint32_t FirstWord = *(uint32_t*)APP_START_ADDR;
    return (FirstWord == 0xFFFFFFFF);
}

void OnError() {
    if(AppIsEmpty()) {
//        Led.StartOrRestart(lsqError); // Display error
        while(true);
    }
    else JumpToApp();
}

static inline void InitClk() {
    FMC->EnableCashAndPrefetch();
    RCU->EnPwrMgmtUnit();
    FMC->SetLatency(48);
    if(RCU->EnableIRC48M() == retv::Ok) {
        RCU->SetCK48MSel_CKIRC48M();   // Use IRC48M as clk src instead of PLL
        RCU->EnCTC();              // Enable trimmer
        // Setup trimmer: SOF freq is 1 kHz, rising front, no prescaler, input is UsbSOF
        CTC->Setup(CTC_REFPOL_RISING, CTC_REFSEL_USBSOF, CTC_REFPSC_DIV_1, 1000);
        // Setup system clock
        RCU->SetPllPresel_CKIRC48M(); // IRC48M is a source
        RCU->SetPrediv0Sel_XTALorIRC48M(); // IRC48M is a source
        // 48MHz div 6 = 8MHz; 8MHz *6 = 48MHz. PLL input freq must be in [1; 25] MHz, 8MHz typical
        RCU->SetPrediv0(6);
        RCU->SetPllSel_Prediv0();
        RCU->SetPllMulti(PllMulti::mul06);
        // Switch clk
        if(RCU->EnablePll() == retv::Ok) {
            RCU->SetAhbPrescaler(AhbPsc::div1);
            RCU->SwitchCkSys2PLL();
        }
    }
    Clk::UpdateFreqValues();
}

void main(void) {
    InitClk();
    // Init peripheral
    RCU->EnAFIO();
    AFIO->DisableJtagDP(); // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST
    Uart.Init();

    // RTOS & Event queue
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk::PrintFreqs();

    Lumos.Init();
//    Lumos.StartOrRestart(lsqFadeIn);

    // Spi Fplash and Msd glue
    AFIO->RemapSPI0_PB345();
    SpiFlash.Init();
    SpiFlash.Reset();
    SpiFlash_t::MemParams_t mp = SpiFlash.GetParams();
    MsdMem::BlockCnt = mp.SectorCnt;
    MsdMem::BlockSz = mp.SectorSz;
    Printf("Flash: %u sectors of %u bytes\r", mp.SectorCnt, mp.SectorSz);

    // Init filesystem
    if(f_mount(&FlashFS, "", 0) != FR_OK) {
        Printf("FS error\r");
        Sys::SleepMilliseconds(99);
        JumpToApp();
    }

#if 1 // =========================== Preparations ==============================
    // Try open file, jump to main app if not found
    if(f_findfirst(&Dir, &FileInfo, "", FILENAME_PATTERN) != FR_OK) {
        Printf("File search fail\r");
        Sys::SleepMilliseconds(99);
        OnError();
    }
    if(FileInfo.fname[0] == 0) {
        Printf("%S not found\r", FILENAME_PATTERN);
        Sys::SleepMilliseconds(99);
        OnError();
    }
    Printf("Found: %S\r", FileInfo.fname);
    if(TryOpenFileRead(FileInfo.fname, &CommonFile) != retv::Ok) OnError();
    int32_t TotalLen = f_size(&CommonFile);
    Flash::UnlockFlash();
#endif
#if 1 // ======= Reading and flashing =======
    Lumos.StartOrRestart(lsqWriting);
    // ==== Read file block by block, do not write first page ====
    uint32_t BytesCnt, CurrentAddr = APP_START_ADDR;
    while(TotalLen != 0) {
        // Check flash address
        if(CurrentAddr >= (FLASH_BASE + TOTAL_FLASH_SZ)) {
            Printf("Error: too large file\r");
            break;
        }

        // How many bytes to read?
        BytesCnt = MIN_(TotalLen, PAGE_SZ);
        TotalLen -= BytesCnt;
        // Read block
        if(f_read(&CommonFile, Buf, BytesCnt, &BytesCnt) != FR_OK) {
            Printf("Read error\r");
            OnError();
        }

        ErasePage(CurrentAddr);
        WriteToMemory(&CurrentAddr, Buf, BytesCnt);
    } // while
#endif
    Printf("\rWriting done\r\n");
    f_close(&CommonFile);
    // Remove firmware file
    f_unlink(FileInfo.fname);
    Flash::LockFlash();
    JumpToApp();

    // Forever
    while(true);



    UsbMsdCdc.Init();
//    UsbMsdCdc.Connect();

    TmrUartCheck.StartOrRestart();

    // Main evt cycle
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case EvtId::UartCheckTime:
                Gpio::Toggle(PA10);
                while(Uart.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&Uart);
                break;

            case EvtId::UsbCdcDataRcvd:
                while(UsbMsdCdc.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&UsbMsdCdc);
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

void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else if(PCmd->NameIs("tst")) {
    }

    else if(PCmd->NameIs("conn")) {
        UsbMsdCdc.Connect();
        PShell->Ok();
    }
    else if(PCmd->NameIs("dsc")) {
        UsbMsdCdc.Disconnect();
        PShell->Ok();
    }

    else PShell->CmdUnknown();
}
