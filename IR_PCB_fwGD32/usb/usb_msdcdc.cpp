/*
 * usb_cdc.cpp
 *
 *  Created on: 03 ����. 2015 �.
 *      Author: Kreyl
 */

#include <descriptors_msdcdc.h>
#include <usb_msdcdc.h>
#include "board.h"
#include "gd_lib.h"
#include "mem_msd_glue.h"
#include "usb.h"
#include "MsgQ.h"
#include "EvtMsgIDs.h"

UsbMsdCdc_t UsbMsdCdc;

static uint8_t SByte;

#define CDC_OUT_BUF_SZ  EP_CDC_BULK_SZ

// MSD related
void OnMSDDataOut();
void OnMSDDataIn();
static bool ISayIsReady = true;
static Thread_t *PMsdThd, *PCdcThd;

// CDC Reception buffers and methods
namespace CdcOutQ {
static enum class TransferAction { SendEvt, WakeThd } Action = TransferAction::SendEvt;

static uint8_t Buf1[CDC_OUT_BUF_SZ], Buf2[CDC_OUT_BUF_SZ], *pBufW = Buf1;
static Buf_t BufToParse;
static Thread_t *pWaitingThd = nullptr;

// OUT transfer end callback
void OnTransferEnd(uint32_t Sz) {
    Sys::LockFromIRQ();
    // Save what received
    BufToParse.Ptr = pBufW;
    BufToParse.Sz = Sz;
    // Switch buffers and start new reception
    pBufW = (pBufW == Buf1)? Buf2 : Buf1;
    Usb::StartReceiveI(EP_CDC_DATA_OUT, pBufW, CDC_OUT_BUF_SZ);
    // Take necessary action
    if(Action == TransferAction::SendEvt) EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId::UsbDataRcvd));
    else Sys::WakeI(&pWaitingThd, retv::Ok);
    Sys::UnlockFromIRQ();
}

} // namespace

// Transmission buffers: several buffers of EP_BULK_SZ
static BufQ_t<uint8_t, EP_CDC_BULK_SZ, USB_TXBUF_CNT> CdcInBuf;

// IN transfer end callback
void CdcOnBulkInTransferEnd() {
    // Unlock the buffer just sent to allow it to be written again
    CdcInBuf.UnlockBuf();
    // Start tx if buf is not empty
    if(Usb::IsActive() and !CdcInBuf.IsEmpty()) {
        BufType_t<uint8_t> buf = CdcInBuf.GetAndLockBuf();
        Sys::LockFromIRQ();
        Usb::StartTransmitI(EP_CDC_DATA_IN, buf.Ptr, buf.Sz);
        Sys::UnlockFromIRQ();
    }
}

#if 1 // ====================== CDC Line Coding related ========================
#define CDC_SET_LINE_CODING         0x20U
#define CDC_GET_LINE_CODING         0x21U
#define CDC_SET_CONTROL_LINE_STATE  0x22U

#define LC_STOP_1                   0U
#define LC_STOP_1P5                 1U
#define LC_STOP_2                   2U

#define LC_PARITY_NONE              0U
#define LC_PARITY_ODD               1U
#define LC_PARITY_EVEN              2U
#define LC_PARITY_MARK              3U
#define LC_PARITY_SPACE             4U

// Line Coding
#define CDC_LINECODING_SZ   7UL
static union CDCLinecoding_t {
    struct {
        uint32_t dwDTERate = 115200;
        uint8_t bCharFormat = LC_STOP_1;
        uint8_t bParityType = LC_PARITY_NONE;
        uint8_t bDataBits = 8;
    };
    uint8_t Buf8[CDC_LINECODING_SZ];
} linecoding;
#endif

#if 1 // ====================== Endpoints config ===============================
// InMultiplier determines the space allocated for the TXFIFO as multiples of the packet size
// ==== EP1 ==== both IN and OUT
static const Usb::EpConfig_t CdcEpBulkCfg = {
        .OutTransferEndCallback = CdcOutQ::OnTransferEnd,
        .InTransferEndCallback = CdcOnBulkInTransferEnd,
        .Type = Usb::EpType::Bulk,
        .OutMaxPktSz = EP_CDC_BULK_SZ,
        .InMaxPktSz = EP_CDC_BULK_SZ,
        .InMultiplier = 2
};

// ==== EP2 ==== Interrupt, IN only. Actually not used.
static const Usb::EpConfig_t CdcEpInterruptCfg = {
        .OutTransferEndCallback = nullptr,
        .InTransferEndCallback = nullptr,
        .Type = Usb::EpType::Interrupt,
        .OutMaxPktSz = 0, // IN only
        .InMaxPktSz = EP_INTERRUPT_SZ,
        .InMultiplier = 1
};

// ==== EP3 ==== both IN and OUT
static const Usb::EpConfig_t MsdEpCfg = {
        .OutTransferEndCallback = OnMSDDataOut,
        .InTransferEndCallback  = OnMSDDataIn,
        .Type = Usb::EpType::Bulk,
        .OutMaxPktSz = EP_MSD_BULK_SZ,
        .InMaxPktSz = EP_MSD_BULK_SZ,
        .InMultiplier = 2
};
#endif

#if 1 // ============================ Events ===================================
// Setup request callback: process class-related requests
Usb::StpReqCbRpl_t Usb::SetupReqHookCallback() {
    Usb::StpReqCbRpl_t Reply; // NotFound by default
    if(Usb::SetupPkt.Type == USB_REQTYPE_CLASS) {
        // === CDC handler ===
        if(Usb::SetupPkt.wIndex == 1) {
            switch(Usb::SetupPkt.bRequest) {
                case CDC_GET_LINE_CODING: // Send linecoding
                    Reply.Retval = retv::Ok;
                    Reply.Buf = linecoding.Buf8;
                    Reply.Sz = CDC_LINECODING_SZ;
                    break;
                case CDC_SET_LINE_CODING: // Receive data into linecoding
                    Reply.Retval = retv::Ok;
                    Reply.Buf= linecoding.Buf8;
                    Reply.Sz = CDC_LINECODING_SZ;
                    break;
                case CDC_SET_CONTROL_LINE_STATE: // Nothing to do, there are no control lines
                    Reply.Retval = retv::Ok;
                    Reply.Sz = 0; // Receive nothing
                    break;
                default: break;
            } // switch
        } // if windex == 1

        // === MSD handler ===
        else {
            // GetMaxLun
            if(Usb::SetupPkt.Direction == USB_REQDIR_DEV2HOST and
                    Usb::SetupPkt.bRequest == MS_REQ_GetMaxLUN and
                    Usb::SetupPkt.wLength == 1) {
//                PrintfI("MS_REQ_GetMaxLUN\r");
                SByte = 0;  // Maximum LUN ID
                Reply.Retval = retv::Ok;
                Reply.Buf = &SByte;
                Reply.Sz = 1;
            }
            // Reset
            else if(Usb::SetupPkt.Direction == USB_REQDIR_HOST2DEV and
                    Usb::SetupPkt.bRequest == MS_REQ_MassStorageReset and
                    Usb::SetupPkt.wLength == 0) {
//                PrintfI("MS_REQ_MassStorageReset\r");
                // TODO: remove Stall condition
                Reply.Retval = retv::Ok; // Acknowledge reception
            }
        } // if MSD
    } // if class
    return Reply;
}

// this callback is invoked from an ISR so I-Class functions must be used
void Usb::EventCallback(Usb::Evt event) {
    switch(event) {
        case Usb::Evt::Reset:
            return;
        case Usb::Evt::Address:
            return;
        case Usb::Evt::Configured:
            Sys::LockFromIRQ();
            // ==== CDC ====
            Usb::InitEp(EP_CDC_DATA_IN,   &CdcEpBulkCfg);
            Usb::InitEp(EP_CDC_INTERRUPT, &CdcEpInterruptCfg);
            // Reset queues
            CdcOutQ::pBufW = (CdcOutQ::pBufW == CdcOutQ::Buf1)? CdcOutQ::Buf2 : CdcOutQ::Buf1;
            // Start reception. In this case, transaction size is limited to EP max size
            Usb::StartReceiveI(EP_CDC_DATA_OUT, CdcOutQ::pBufW, CDC_OUT_BUF_SZ);
            // ==== MSD ====
            Usb::InitEp(EP_MSD_DATA_IN,   &MsdEpCfg);
            ISayIsReady = true;
            // ==== Sys ====
            EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId::UsbReady)); // Inform main thread
            Sys::UnlockFromIRQ();
            return;
        case Usb::Evt::Suspend:
        case Usb::Evt::Wakeup:
        case Usb::Evt::Stalled:
        case Usb::Evt::Unconfigured:
            return;
    } // switch
}
#endif // Events

#if 1 // =========================== CDC =======================================
retv UsbMsdCdc_t::IPutChar(char c) {
    if(!Usb::IsActive()) return retv::Disconnected;
    retv r = CdcInBuf.Put(c);
    if(CdcInBuf.IsFullBufPresent() and !Usb::IsEpTransmitting(EP_CDC_DATA_IN)) { // New buffer is full
        BufType_t<uint8_t> buf = CdcInBuf.GetAndLockBuf();
        Usb::StartTransmit(EP_CDC_DATA_IN, buf.Ptr, buf.Sz);
    }
    return r;
}

void UsbMsdCdc_t::IStartTransmissionIfNotYet() {
    // Start tx if it has not already started and if buf is not empty.
    if(Usb::IsActive() and !Usb::IsEpTransmitting(EP_CDC_DATA_IN) and !CdcInBuf.IsEmpty()) {
        BufType_t<uint8_t> buf = CdcInBuf.GetAndLockBuf();
        Usb::StartTransmit(EP_CDC_DATA_IN, buf.Ptr, buf.Sz);
    }
}

retv UsbMsdCdc_t::TryParseRxBuff() {
    while(CdcOutQ::BufToParse.Sz) {
        CdcOutQ::BufToParse.Sz--;
        if(Cmd.PutChar(*CdcOutQ::BufToParse.Ptr++) == pdrNewCmd) return retv::Ok;
    }
    return retv::Fail;
}

// Send '>' and receive what follows
retv UsbMsdCdc_t::ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {
    CdcOutQ::Action = CdcOutQ::TransferAction::WakeThd; // Do not send evt to main q on buf reception
    if(IPutChar('>') != retv::Ok) return retv::Fail;
    IStartTransmissionIfNotYet();
    // Wait for data to be received
    Sys::Lock();
    systime_t Start = Sys::GetSysTimeX();
    systime_t TimeLeft, Timeout_st = TIME_MS2I(Timeout_ms);
    while(Len != 0) {
        // Calculate time left to wait
        systime_t Elapsed = Sys::TimeElapsedSince(Start);
        if(Elapsed > Timeout_st) break;
        TimeLeft = Timeout_st - Elapsed;
        // Wait data
        CdcOutQ::pWaitingThd = Sys::GetSelfThd();
        if(Sys::SleepS(TimeLeft) == retv::Timeout) break; // Timeout occured
        // Will be here after successful reception; put data to buffer
        if(CdcOutQ::BufToParse.Sz > Len) CdcOutQ::BufToParse.Sz = Len; // Flush too large data
        memcpy(ptr, CdcOutQ::BufToParse.Ptr, CdcOutQ::BufToParse.Sz);
        Len -= CdcOutQ::BufToParse.Sz;
        ptr += CdcOutQ::BufToParse.Sz;
    }
    CdcOutQ::Action = CdcOutQ::TransferAction::SendEvt; // Return to normal life
    Sys::Unlock();
    return (Len == 0)? retv::Ok : retv::Fail; // Check if everything was received
}

// Wait '>' and then transmit buffer
retv UsbMsdCdc_t::TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) {
    if(Usb::IsEpTransmitting(EP_CDC_DATA_IN)) return retv::Busy;
    retv r = retv::Timeout;
    Sys::Lock();
    CdcOutQ::Action = CdcOutQ::TransferAction::WakeThd; // Do not send evt to main q on buf reception
    systime_t Start = Sys::GetSysTimeX();
    systime_t TimeLeft, Timeout_st = TIME_MS2I(Timeout_ms);
    // Wait '>'
    while(r == retv::Timeout) {
        // Calculate time left to wait
        systime_t Elapsed = Sys::TimeElapsedSince(Start);
        if(Elapsed > Timeout_st) break;
        TimeLeft = Timeout_st - Elapsed;
        // Wait data
        CdcOutQ::pWaitingThd = Sys::GetSelfThd();
        if(Sys::SleepS(TimeLeft) == retv::Timeout) break; // Timeout occured
        // Will be here after successful reception; check if '>' present
        for(uint32_t i=0; i<CdcOutQ::BufToParse.Sz; i++) {
            if(CdcOutQ::BufToParse.Ptr[i] == '>') {
                // Found
                r = retv::Ok;
                break;
            }
        } // for
    }
    // Will be here after either timeout or successful '>' reception
    CdcOutQ::Action = CdcOutQ::TransferAction::SendEvt; // Return to normal life
    if(r == retv::Ok) { // Transmit data
        if(Usb::IsActive()) Usb::StartTransmitI(EP_CDC_DATA_IN, ptr, Len);
        else r = retv::Disconnected;
    }
    Sys::Unlock();
    return r;
}
#endif



#define EVT_USB_READY           EVENT_MASK(10)
#define EVT_USB_RESET           EVENT_MASK(11)
#define EVT_USB_SUSPEND         EVENT_MASK(12)
#define EVT_USB_CONNECTED       EVENT_MASK(13)
#define EVT_USB_DISCONNECTED    EVENT_MASK(14)
#define EVT_USB_IN_DONE         EVENT_MASK(15)
#define EVT_USB_OUT_DONE        EVENT_MASK(16)

//void OnMSDDataIn(USBDriver *usbp, usbep_t ep) {
//    chSysLockFromISR();
//    chEvtSignalI(PMsdThd, EVT_USB_IN_DONE);
//    chSysUnlockFromISR();
//}
//
//void OnMSDDataOut(USBDriver *usbp, usbep_t ep) {
//    chSysLockFromISR();
//    chEvtSignalI(PMsdThd, EVT_USB_OUT_DONE);
//    chSysUnlockFromISR();
//}


#if 1 // ========================== MSD Thread =================================
static MS_CommandBlockWrapper_t CmdBlock;
static MS_CommandStatusWrapper_t CmdStatus;
static SCSI_RequestSenseResponse_t SenseData;
static SCSI_ReadCapacity10Response_t ReadCapacity10Response;
static SCSI_ReadFormatCapacitiesResponse_t ReadFormatCapacitiesResponse;
static uint32_t Buf32[(MSD_DATABUF_SZ/4)];

static void SCSICmdHandler();
// Scsi commands
static void CmdTestReady();
static uint8_t CmdStartStopUnit();
static uint8_t CmdInquiry();
static uint8_t CmdRequestSense();
static uint8_t CmdReadCapacity10();
static uint8_t CmdSendDiagnostic();
static uint8_t CmdReadFormatCapacities();
static uint8_t CmdRead10();
static uint8_t CmdWrite10();
static uint8_t CmdModeSense6();
static uint8_t ReadWriteCommon(uint32_t *PAddr, uint16_t *PLen);
static void BusyWaitIN();
static uint8_t BusyWaitOUT();
//static void TransmitBuf(uint8_t *Ptr, uint32_t Len);
//static uint8_t ReceiveToBuf(uint8_t *Ptr, uint32_t Len);

static THD_WORKING_AREA(waUsbThd, 128);
static THD_FUNCTION(UsbThd, arg) {
    chRegSetThreadName("Usb");
    while(true) {
        uint32_t EvtMsk = chEvtWaitAny(ALL_EVENTS);
        if(EvtMsk & EVT_USB_READY) {
            // Receive header
            chSysLock();
            usbStartReceiveI(&USBDrv, EP_MSD_OUT_ID, (uint8_t*)&CmdBlock, MS_CMD_SZ);
            chSysUnlock();
        }

        if(EvtMsk & EVT_USB_OUT_DONE) {
            SCSICmdHandler();
            // Receive header again
            chSysLock();
            usbStartReceiveI(&USBDrv, EP_MSD_OUT_ID, (uint8_t*)&CmdBlock, MS_CMD_SZ);
            chSysUnlock();
        }
    } // while true
}
#endif

#if 1 // ========================== RX Thread ==================================
bool UsbMsdCdc_t::IsActive() { return (SDU1.config->usbp->state == USB_ACTIVE); }

static THD_WORKING_AREA(waThdCDCRX, 128);
static THD_FUNCTION(ThdCDCRX, arg) {
    chRegSetThreadName("CDCRX");
    while(true) {
        if(UsbMsdCdc.IsActive()) {
            msg_t m = SDU1.vmt->get(&SDU1);
            if(m > 0) {
//                SDU1.vmt->put(&SDU1, (uint8_t)m);   // repeat what was sent
                if(UsbMsdCdc.Cmd.PutChar((char)m) == pdrNewCmd) {
                    chSysLock();
                    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdShellCmd, (Shell_t*)&UsbMsdCdc));
                    chSchGoSleepS(CH_STATE_SUSPENDED); // Wait until cmd processed
                    chSysUnlock();  // Will be here when application signals that cmd processed
                }
            } // if >0
        } // if active
        else chThdSleepMilliseconds(540);
    } // while true
}

uint8_t UsbMsdCdc_t::IPutChar(char c) {
    return (SDU1.vmt->put(&SDU1, (uint8_t)c) == MSG_OK)? retvOk : retvFail;
}

void UsbMsdCdc_t::SignalCmdProcessed() {
    chSysLock();
    if(PCdcThd->state == CH_STATE_SUSPENDED) chSchReadyI(PCdcThd);
    chSysUnlock();
}
#endif

void UsbMsdCdc_t::Init() {

    // Variables
    SenseData.ResponseCode = 0x70;
    SenseData.AddSenseLen = 0x0A;
    // MSD Thread
    PMsdThd = chThdCreateStatic(waUsbThd, sizeof(waUsbThd), NORMALPRIO, (tfunc_t)UsbThd, NULL);
    // Objects
    usbInit();
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &SerUsbCfg);
    // CDC thread
    PCdcThd = chThdCreateStatic(waThdCDCRX, sizeof(waThdCDCRX), NORMALPRIO, ThdCDCRX, NULL);
}

void UsbMsdCdc_t::Reset() {
    // Wake thread if sleeping
    chSysLock();
    if(PMsdThd->state == CH_STATE_SUSPENDED) chSchReadyI(PMsdThd);
    chSysUnlock();
}

void UsbMsdCdc_t::Connect() {
#if defined STM32F1XX
    // Disconnect everything
    PinSetupAnalog(USB_PULLUP);
    PinSetupAnalog(USB_DM);
    PinSetupAnalog(USB_DP);
#else
    usbDisconnectBus(SerUsbCfg.usbp);
#endif
    chThdSleepMilliseconds(99);
    usbStart(SerUsbCfg.usbp, &UsbCfg);
#if defined STM32F1XX
    PinSetupAlterFunc(USB_DM, omPushPull, pudNone, USB_AF, psHigh);
    PinSetupAlterFunc(USB_DP, omPushPull, pudNone, USB_AF, psHigh);
    PinSetHi(USB_PULLUP);
    PinSetupOut(USB_PULLUP, omPushPull);
#else
    usbConnectBus(SerUsbCfg.usbp);
#endif
}
void UsbMsdCdc_t::Disconnect() {
    usbStop(SerUsbCfg.usbp);
#if defined STM32F1XX
    // Disconnect everything
    PinSetupAnalog(USB_PULLUP);
    PinSetupAnalog(USB_DM);
    PinSetupAnalog(USB_DP);
#else
    usbDisconnectBus(SerUsbCfg.usbp);
#endif
}

void TransmitBuf(uint32_t *Ptr, uint32_t Len) {
    chSysLock();
    usbStartTransmitI(&USBDrv, EP_MSD_IN_ID, (uint8_t*)Ptr, Len);
    chSysUnlock();
    BusyWaitIN();
}

uint8_t ReceiveToBuf(uint32_t *Ptr, uint32_t Len) {
    chSysLock();
    usbStartReceiveI(&USBDrv, EP_MSD_IN_ID, (uint8_t*)Ptr, Len);
    chSysUnlock();
    return BusyWaitOUT();
}

void BusyWaitIN() {
    chEvtWaitAny(ALL_EVENTS);
}

uint8_t BusyWaitOUT() {
    eventmask_t evt = chEvtWaitAnyTimeout(EVT_USB_OUT_DONE, TIME_MS2I(MSD_TIMEOUT_MS));
    return (evt == 0)? retvTimeout : retvOk;
}

#if 1 // =========================== SCSI ======================================
//#define DBG_PRINT_CMD   TRUE
void SCSICmdHandler() {
//    Uart.Printf("Sgn=%X; Tag=%X; Len=%u; Flags=%X; LUN=%u; SLen=%u; SCmd=%A\r", CmdBlock.Signature, CmdBlock.Tag, CmdBlock.DataTransferLen, CmdBlock.Flags, CmdBlock.LUN, CmdBlock.SCSICmdLen, CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
//    Printf("SCmd=%A\r", CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
    uint8_t CmdRslt = retvFail;
    switch(CmdBlock.SCSICmdData[0]) {
        case SCSI_CMD_TEST_UNIT_READY:    CmdTestReady();     return; break;    // Will report itself
        case SCSI_CMD_START_STOP_UNIT:    CmdRslt = CmdStartStopUnit(); break;
        case SCSI_CMD_INQUIRY:            CmdRslt = CmdInquiry(); break;
        case SCSI_CMD_REQUEST_SENSE:      CmdRslt = CmdRequestSense(); break;
        case SCSI_CMD_READ_CAPACITY_10:   CmdRslt = CmdReadCapacity10(); break;
        case SCSI_CMD_SEND_DIAGNOSTIC:    CmdRslt = CmdSendDiagnostic(); break;
        case SCSI_READ_FORMAT_CAPACITIES: CmdRslt = CmdReadFormatCapacities(); break;
        case SCSI_CMD_WRITE_10:           CmdRslt = CmdWrite10(); break;
        case SCSI_CMD_READ_10:            CmdRslt = CmdRead10(); break;
        case SCSI_CMD_MODE_SENSE_6:       CmdRslt = CmdModeSense6(); break;
        // These commands should just succeed, no handling required
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case SCSI_CMD_VERIFY_10:
        case SCSI_CMD_SYNCHRONIZE_CACHE_10:
            CmdRslt = retvOk;
            CmdBlock.DataTransferLen = 0;
            break;
        default:
            Printf("MSCmd %X not supported\r", CmdBlock.SCSICmdData[0]);
            // Update the SENSE key to reflect the invalid command
            SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
            SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
            SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
            break;
    } // switch
    // Update Sense if command was successfully processed
    if(CmdRslt == retvOk) {
        SenseData.SenseKey = SCSI_SENSE_KEY_GOOD;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    // Send status
    CmdStatus.Signature = MS_CSW_SIGNATURE;
    CmdStatus.Tag = CmdBlock.Tag;
    if(CmdRslt == retvOk) {
        CmdStatus.Status = SCSI_STATUS_OK;
        // DataTransferLen decreased at cmd handlers
        CmdStatus.DataTransferResidue = CmdBlock.DataTransferLen;
    }
    else {
        CmdStatus.Status = SCSI_STATUS_CHECK_CONDITION;
        CmdStatus.DataTransferResidue = 0;    // 0 or requested length?
    }

    // Stall if cmd failed and there is data to send
//    bool ShouldSendStatus = true;
//    if((CmdRslt != retvOk)) {
//        chSysLock();
//        ShouldSendStatus = !usbStallTransmitI(&USBDrv, EP_DATA_IN_ID);  // transmit status if successfully stalled
//        chSysUnlock();
//    }
//    if(ShouldSendStatus) {
        TransmitBuf((uint32_t*)&CmdStatus, sizeof(MS_CommandStatusWrapper_t));
//    }
}

void CmdTestReady() {
#if DBG_PRINT_CMD
    Printf("CmdTestReady (Rdy: %u)\r", ISayIsReady);
#endif
    CmdBlock.DataTransferLen = 0;
    CmdStatus.Signature = MS_CSW_SIGNATURE;
    CmdStatus.Tag = CmdBlock.Tag;
    CmdStatus.DataTransferResidue = CmdBlock.DataTransferLen;

    if(ISayIsReady) {
        CmdStatus.Status = SCSI_STATUS_OK;
        SenseData.SenseKey = SCSI_SENSE_KEY_GOOD;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }
    else {
        CmdStatus.Status = SCSI_STATUS_CHECK_CONDITION;
        SenseData.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    TransmitBuf((uint32_t*)&CmdStatus, sizeof(MS_CommandStatusWrapper_t));
}

uint8_t CmdStartStopUnit() {
#if DBG_PRINT_CMD
    Printf("CmdStartStopUnit [4]=%02X\r", CmdBlock.SCSICmdData[4]);
#endif
    if((CmdBlock.SCSICmdData[4] & 0x03) == 0x02) {  // Eject
        ISayIsReady = false;
    }
    else if((CmdBlock.SCSICmdData[4] & 0x03) == 0x03) {  // Load
        ISayIsReady = true;
    }
    return retvOk;
}

uint8_t CmdInquiry() {
#if DBG_PRINT_CMD
    Printf("CmdInquiry %u\r", CmdBlock.SCSICmdData[1] & 0x01);
#endif
    uint16_t RequestedLength = Convert::BuildUint16(CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3]);
    uint16_t BytesToTransfer;
    if(CmdBlock.SCSICmdData[1] & 0x01) { // Evpd is set
        BytesToTransfer = MIN_(RequestedLength, PAGE0_INQUIRY_DATA_SZ);
        TransmitBuf((uint32_t*)&Page00InquiryData, BytesToTransfer);
    }
    else {
        // Transmit InquiryData
        BytesToTransfer = MIN_(RequestedLength, sizeof(SCSI_InquiryResponse_t));
        TransmitBuf((uint32_t*)&InquiryData, BytesToTransfer);
    }
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retvOk;
}
uint8_t CmdRequestSense() {
#if DBG_PRINT_CMD
    Printf("CmdRequestSense\r");
#endif
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN_(RequestedLength, sizeof(SenseData));
    // Transmit SenceData
    TransmitBuf((uint32_t*)&SenseData, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retvOk;
}
uint8_t CmdReadCapacity10() {
#if DBG_PRINT_CMD
    Printf("CmdReadCapacity10\r");
#endif
    ReadCapacity10Response.LastBlockAddr = __REV((uint32_t)MSD_BLOCK_CNT - 1);
    ReadCapacity10Response.BlockSize = __REV((uint32_t)MSD_BLOCK_SZ);
    // Transmit SenceData
    TransmitBuf((uint32_t*)&ReadCapacity10Response, sizeof(ReadCapacity10Response));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadCapacity10Response);
    return retvOk;
}
uint8_t CmdSendDiagnostic() {
    Printf("CmdSendDiagnostic\r");
    return retvCmdUnknown;
}
uint8_t CmdReadFormatCapacities() {
#if DBG_PRINT_CMD
    Printf("CmdReadFormatCapacities\r");
#endif
    ReadFormatCapacitiesResponse.Length = 0x08;
    ReadFormatCapacitiesResponse.NumberOfBlocks = __REV(MSD_BLOCK_CNT);
    // 01b Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b Formatted Media - Current media capacity
    // 11b No Cartridge in Drive - Maximum formattable capacity
    ReadFormatCapacitiesResponse.DescCode = 0x02;
    ReadFormatCapacitiesResponse.BlockSize[0] = (uint8_t)((uint32_t)MSD_BLOCK_SZ >> 16);
    ReadFormatCapacitiesResponse.BlockSize[1] = (uint8_t)((uint32_t)MSD_BLOCK_SZ >> 8);
    ReadFormatCapacitiesResponse.BlockSize[2] = (uint8_t)((uint32_t)MSD_BLOCK_SZ);
    // Transmit Data
    TransmitBuf((uint32_t*)&ReadFormatCapacitiesResponse, sizeof(ReadFormatCapacitiesResponse));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadFormatCapacitiesResponse);
    return retvOk;
}

uint8_t ReadWriteCommon(uint32_t *PAddr, uint16_t *PLen) {
    *PAddr = Convert::BuildUint32(CmdBlock.SCSICmdData[5], CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3], CmdBlock.SCSICmdData[2]);
    *PLen  = Convert::BuildUint16(CmdBlock.SCSICmdData[8], CmdBlock.SCSICmdData[7]);
//    Uart.Printf("Addr=%u; Len=%u\r", *PAddr, *PLen);
    // Check block addr
    if((*PAddr + *PLen) > MSD_BLOCK_CNT) {
        Printf("Out Of Range: Addr %u, Len %u\r", *PAddr, *PLen);
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        return retvFail;
    }
    // Check cases 4, 5: (Hi != Dn); and 3, 11, 13: (Hn, Ho != Do)
    if(CmdBlock.DataTransferLen != (*PLen) * MSD_BLOCK_SZ) {
        Printf("Wrong length\r");
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        return retvFail;
    }
    return retvOk;
}

uint8_t CmdRead10() {
#if DBG_PRINT_CMD
    Printf("CmdRead10\r");
#endif
    uint32_t BlockAddress=0;
    uint16_t TotalBlocks=0;
    if(ReadWriteCommon(&BlockAddress, &TotalBlocks) != retvOk) return retvFail;
    // ==== Send data ====
    uint32_t BlocksToRead, BytesToSend; // Intermediate values
    bool Rslt;
    while(TotalBlocks != 0) {
        BlocksToRead = MIN_(MSD_DATABUF_SZ / MSD_BLOCK_SZ, TotalBlocks);
        BytesToSend = BlocksToRead * MSD_BLOCK_SZ;
        Rslt = MSDRead(BlockAddress, Buf32, BlocksToRead);
//        Uart.Printf("%A\r", Buf, 50, ' ');
        if(Rslt == retvOk) {
            TransmitBuf(Buf32, BytesToSend);
            CmdBlock.DataTransferLen -= BytesToSend;
            TotalBlocks  -= BlocksToRead;
            BlockAddress += BlocksToRead;
        }
        else {
            Printf("Rd fail\r");
            // TODO: handle read error
            return retvFail;
        }
    } // while
    return retvOk;
}

uint8_t CmdWrite10() {
#if DBG_PRINT_CMD
    Printf("CmdWrite10\r");
#endif
#if READ_ONLY
    SenseData.SenseKey = SCSI_SENSE_KEY_DATA_PROTECT;
    SenseData.AdditionalSenseCode = SCSI_ASENSE_WRITE_PROTECTED;
    SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    return retvFail;
#else
    // Check case 8: Hi != Do
    if(CmdBlock.Flags & 0x80) {
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        return retvFail;
    }
    // TODO: Check if ready
    if(false) {
        SenseData.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        return retvFail;
    }
    uint32_t BlockAddress=0;
    uint16_t TotalBlocks=0;
    // Get transaction size
    if(ReadWriteCommon(&BlockAddress, &TotalBlocks) != retvOk) return retvFail;
//    Uart.Printf("Addr=%u; Len=%u\r", BlockAddress, TotalBlocks);
    uint32_t BlocksToWrite, BytesToReceive;
    uint8_t Rslt = retvOk;

    while(TotalBlocks != 0) {
        // Fill Buf1
        BytesToReceive = MIN_(MSD_DATABUF_SZ, TotalBlocks * MSD_BLOCK_SZ);
        BlocksToWrite  = BytesToReceive / MSD_BLOCK_SZ;
        if(ReceiveToBuf(Buf32, BytesToReceive) != retvOk) {
            Printf("Rcv fail\r");
            return retvFail;
        }
        // Write Buf to memory
        Rslt = MSDWrite(BlockAddress, Buf32, BlocksToWrite);
        if(Rslt != retvOk) {
            Printf("Wr fail\r");
            return retvFail;
        }
        CmdBlock.DataTransferLen -= BytesToReceive;
        TotalBlocks -= BlocksToWrite;
        BlockAddress += BlocksToWrite;
    } // while
    return retvOk;
#endif
}

uint8_t CmdModeSense6() {
#if DBG_PRINT_CMD
    Printf("CmdModeSense6\r");
#endif
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN_(RequestedLength, MODE_SENSE6_DATA_SZ);
    TransmitBuf((uint32_t*)&Mode_Sense6_data, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retvOk;
}
#endif
