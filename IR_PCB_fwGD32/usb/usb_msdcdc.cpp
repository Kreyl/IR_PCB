/*
 * usb_cdc.cpp
 *
 *  Created on: 03 ����. 2015 �.
 *      Author: Kreyl
 */

#include "descriptors_msdcdc.h"
#include "usb_msdcdc.h"
#include "board.h"
#include "gd_lib.h"
#include "mem_msd_glue.h"
#include "usb.h"
#include "scsi.h"
#include "MsgQ.h"
#include "EvtMsgIDs.h"

UsbMsdCdc_t UsbMsdCdc;

static uint8_t SByte;

#define CDC_OUT_BUF_SZ  EP_CDC_BULK_SZ

#if 1 // ============ Mass Storage constants, types, variables =================
// Enum for the Mass Storage class specific control requests that can be issued by the USB bus host
enum MS_ClassRequests_t {
    // Mass Storage class-specific request to retrieve the total number of Logical Units (drives) in the SCSI device.
    MS_REQ_GetMaxLUN = 0xFE,
    // Mass Storage class-specific request to reset the Mass Storage interface, ready for the next command.
    MS_REQ_MassStorageReset = 0xFF,
};

#pragma pack(push, 1)
// Mass Storage Class Command Block Wrapper
struct MS_CommandBlockWrapper_t {
    uint32_t Signature;         // Command block signature, must be MS_CBW_SIGNATURE to indicate a valid Command Block
    uint32_t Tag;               // Unique command ID value, to associate a command block wrapper with its command status wrapper
    uint32_t DataTransferLen;   // Length of the optional data portion of the issued command, in bytes
    uint8_t  Flags;             // Command block flags, indicating command data direction
    uint8_t  LUN;               // Logical Unit number this command is issued to
    uint8_t  SCSICmdLen;        // Length of the issued SCSI command within the SCSI command data array
    uint8_t  SCSICmdData[16];   // Issued SCSI command in the Command Block
};
#define MS_CMD_SZ   sizeof(MS_CommandBlockWrapper_t)

// Mass Storage Class Command Status Wrapper
struct MS_CommandStatusWrapper_t {
    uint32_t Signature;          // Status block signature, must be \ref MS_CSW_SIGNATURE to indicate a valid Command Status
    uint32_t Tag;                // Unique command ID value, to associate a command block wrapper with its command status wrapper
    uint32_t DataTransferResidue;// Number of bytes of data not processed in the SCSI command
    uint8_t  Status;             // Status code of the issued command - a value from the MS_CommandStatusCodes_t enum
};
#pragma pack(pop)

void MSDStartReceiveHdrI();
void OnMSDDataOut(uint32_t Sz);
void OnMSDDataIn();
static bool ISayIsReady = true;
static Thread_t *PMsdThd = nullptr;
#endif

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
    Usb::StartReceiveI(EP_CDC_DATA, pBufW, CDC_OUT_BUF_SZ);
    // Take necessary action
    if(Action == TransferAction::SendEvt) EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId::UsbCdcDataRcvd));
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
        Usb::StartTransmitI(EP_CDC_DATA, buf.Ptr, buf.Sz);
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
            Usb::InitEp(EP_CDC_DATA,   &CdcEpBulkCfg);
            Usb::InitEp(EP_CDC_INTERRUPT, &CdcEpInterruptCfg);
            // Reset queues
            CdcOutQ::pBufW = (CdcOutQ::pBufW == CdcOutQ::Buf1)? CdcOutQ::Buf2 : CdcOutQ::Buf1;
            // Start reception. In this case, transaction size is limited to EP max size
            Usb::StartReceiveI(EP_CDC_DATA, CdcOutQ::pBufW, CDC_OUT_BUF_SZ);
            // ==== MSD ====
            Usb::InitEp(EP_MSD_DATA,   &MsdEpCfg);
            MSDStartReceiveHdrI();
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
    Sys::Lock();
    retv r = CdcInBuf.Put(c);
    if(CdcInBuf.IsFullBufPresent() and !Usb::IsEpTransmitting(EP_CDC_DATA)) { // New buffer is full
        BufType_t<uint8_t> buf = CdcInBuf.GetAndLockBuf();
        Usb::StartTransmitI(EP_CDC_DATA, buf.Ptr, buf.Sz);
    }
    Sys::Unlock();
    return r;
}

void UsbMsdCdc_t::IStartTransmissionIfNotYet() {
    // Start tx if it has not already started and if buf is not empty.
    if(Usb::IsActive() and !Usb::IsEpTransmitting(EP_CDC_DATA) and !CdcInBuf.IsEmpty()) {
        Sys::Lock();
        BufType_t<uint8_t> buf = CdcInBuf.GetAndLockBuf();
        Usb::StartTransmitI(EP_CDC_DATA, buf.Ptr, buf.Sz);
        Sys::Unlock();
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
    if(Usb::IsEpTransmitting(EP_CDC_DATA)) return retv::Busy;
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
        if(Usb::IsActive()) Usb::StartTransmitI(EP_CDC_DATA, ptr, Len);
        else r = retv::Disconnected;
    }
    Sys::Unlock();
    return r;
}
#endif

#if 1 // ============================= MSD =====================================
//#define DBG_PRINT_CMD   TRUE

static MS_CommandBlockWrapper_t CmdBlock;
static MS_CommandStatusWrapper_t CmdStatus;
static SCSI_RequestSenseResponse_t SenseData;
static SCSI_ReadCapacity10Response_t ReadCapacity10Response;
static SCSI_ReadFormatCapacitiesResponse_t ReadFormatCapacitiesResponse;
static uint32_t Buf32[(MSD_DATABUF_SZ/4)];

static void SCSICmdHandler();
// Scsi commands
static void CmdTestUnitReady();
static retv CmdStartStopUnit();
static retv CmdInquiry();
static retv CmdRequestSense();
static retv CmdReadCapacity10();
static retv CmdSendDiagnostic();
static retv CmdReadFormatCapacities();
static retv CmdRead10();
static retv CmdWrite10();
static retv CmdModeSense6();

// ==== Thread ====
static THD_WORKSPACE(waMsdThd, 256);
__attribute__((noreturn)) static void MsdThd() {
    while(true) {
        Sys::Lock();
        PMsdThd = Sys::GetSelfThd();
        retv r = Sys::SleepS(TIME_INFINITE); // Wait forever until new data is received
        Sys::Unlock();
        if(r == retv::Ok) SCSICmdHandler(); // New header received
        Sys::Lock();
        MSDStartReceiveHdrI();
        Sys::Unlock();
    }
}

// Receive header
void MSDStartReceiveHdrI() {
    Usb::StartReceiveI(EP_MSD_DATA, (uint8_t*)&CmdBlock, MS_CMD_SZ);
}

void OnMSDDataOut(uint32_t Sz) {
    Sys::LockFromIRQ();
    if(PMsdThd and PMsdThd->state == ThdState::Sleeping) Sys::WakeI(&PMsdThd, retv::Ok);
    Sys::UnlockFromIRQ();
}

void OnMSDDataIn() {
    Sys::LockFromIRQ();
    if(PMsdThd and PMsdThd->state == ThdState::Sleeping) Sys::WakeI(&PMsdThd, retv::Ok);
    Sys::UnlockFromIRQ();
}

void TransmitBuf(uint32_t *Ptr, uint32_t Len) {
    Sys::Lock();
    PMsdThd = Sys::GetSelfThd();
    Usb::StartTransmitI(EP_MSD_DATA, (uint8_t*)Ptr, Len);
    Sys::SleepS(TIME_INFINITE); // Wait forever until data is transmitted
    Sys::Unlock();
}

retv ReceiveToBuf(uint32_t *Ptr, uint32_t Len) {
    Sys::Lock();
    PMsdThd = Sys::GetSelfThd();
    Usb::StartReceiveI(EP_MSD_DATA, (uint8_t*)Ptr, Len);
    retv r = Sys::SleepS(TIME_INFINITE); // Wait forever until data is received
    Sys::Unlock();
    return r;
}
#endif

void UsbMsdCdc_t::Init() {
    // Variables
    SenseData.ResponseCode = 0x70;
    SenseData.AddSenseLen = 0x0A;
    // MSD Thread
    Sys::CreateThd(waMsdThd, sizeof(waMsdThd), NORMALPRIO, MsdThd);
}

void UsbMsdCdc_t::Reset() {
    // Wake thread if sleeping
    Sys::Lock();
    if(PMsdThd and PMsdThd->state == ThdState::Sleeping) Sys::WakeI(&PMsdThd, retv::Reset);
    Sys::Unlock();
}

void UsbMsdCdc_t::Connect()    { Usb::Connect(); }
void UsbMsdCdc_t::Disconnect() { Usb::Disconnect(); }
bool UsbMsdCdc_t::IsActive()   { return Usb::IsActive(); }

#if 1 // =========================== SCSI ======================================
//#define DBG_PRINT_CMD   TRUE
void SCSICmdHandler() {
//    Printf("Sgn=%X; Tag=%X; Len=%u; Flags=%X; LUN=%u; SLen=%u; SCmd=%A\r", CmdBlock.Signature, CmdBlock.Tag, CmdBlock.DataTransferLen, CmdBlock.Flags, CmdBlock.LUN, CmdBlock.SCSICmdLen, CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
//    Printf("SCmd=%A\r", CmdBlock.SCSICmdData, CmdBlock.SCSICmdLen, ' ');
    retv CmdRslt = retv::Fail;
    switch(CmdBlock.SCSICmdData[0]) {
        case 0x00: CmdTestUnitReady(); return; // Will report inside
        case 0x03: CmdRslt = CmdRequestSense(); break;
        case 0x12: CmdRslt = CmdInquiry(); break;
        case 0x1A: CmdRslt = CmdModeSense6(); break;
        case 0x1B: CmdRslt = CmdStartStopUnit(); break;
        case 0x1D: CmdRslt = CmdSendDiagnostic(); break;
        case 0x23: CmdRslt = CmdReadFormatCapacities(); break;
        case 0x25: CmdRslt = CmdReadCapacity10(); break;
        case 0x28: CmdRslt = CmdRead10(); break;
        case 0x2A: CmdRslt = CmdWrite10(); break;
        // These commands should just succeed, no handling required
        case 0x1E: // Prevent/Allow Medium Removal
        case 0x2F: // Verify10
        case 0x35: // SynchronizeCache10
            CmdRslt = retv::Ok;
            CmdBlock.DataTransferLen = 0;
            break;
        // Not implemented
        case 0x15: // ModeSelect6
        case 0x55: // ModeSelect10
        case 0x5A: // ModeSense10
        case 0xA0: // ReportLUNs
        default:
            Printf("MSCmd %X not supported\r", CmdBlock.SCSICmdData[0]);
            // Update the SENSE key to reflect the invalid command
            SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
            SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
            SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
            break;
    } // switch
    // Update Sense if command was successfully processed
    if(CmdRslt == retv::Ok) {
        SenseData.SenseKey = SCSI_SENSE_KEY_GOOD;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_NO_ADDITIONAL_INFORMATION;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    }

    // Send status
    CmdStatus.Signature = MS_CSW_SIGNATURE;
    CmdStatus.Tag = CmdBlock.Tag;
    if(CmdRslt == retv::Ok) {
        CmdStatus.Status = SCSI_STATUS_OK;
        CmdStatus.DataTransferResidue = CmdBlock.DataTransferLen; // DataTransferLen decreased by cmd handler
    }
    else {
        CmdStatus.Status = SCSI_STATUS_CHECK_CONDITION;
        CmdStatus.DataTransferResidue = 0;    // 0 or requested length?
    }
    TransmitBuf((uint32_t*)&CmdStatus, sizeof(MS_CommandStatusWrapper_t));
}

void CmdTestUnitReady() {
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

retv CmdStartStopUnit() {
#if DBG_PRINT_CMD
    Printf("CmdStartStopUnit [4]=%02X\r", CmdBlock.SCSICmdData[4]);
#endif
    if((CmdBlock.SCSICmdData[4] & 0x03) == 0x02) {  // Eject
        ISayIsReady = false;
    }
    else if((CmdBlock.SCSICmdData[4] & 0x03) == 0x03) {  // Load
        ISayIsReady = true;
    }
    return retv::Ok;
}

retv CmdInquiry() {
#if DBG_PRINT_CMD
    Printf("CmdInquiry %u\r", CmdBlock.SCSICmdData[1] & 0x01);
#endif
    uint16_t RequestedLength =  Convert::BuildU16(CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3]);
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
    return retv::Ok;
}

retv CmdRequestSense() {
#if DBG_PRINT_CMD
    Printf("CmdRequestSense\r");
#endif
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN_(RequestedLength, sizeof(SenseData));
    // Transmit SenceData
    TransmitBuf((uint32_t*)&SenseData, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retv::Ok;
}

retv CmdReadCapacity10() {
#if DBG_PRINT_CMD
    Printf("CmdReadCapacity10\r");
#endif
    ReadCapacity10Response.LastBlockAddr = __REV(MsdMem::BlockCnt - 1);
    ReadCapacity10Response.BlockSize = __REV(MsdMem::BlockSz);
    // Transmit SenceData
    TransmitBuf((uint32_t*)&ReadCapacity10Response, sizeof(ReadCapacity10Response));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadCapacity10Response);
    return retv::Ok;
}

retv CmdSendDiagnostic() {
    Printf("CmdSendDiagnostic\r");
    return retv::CmdUnknown;
}

retv CmdReadFormatCapacities() {
#if DBG_PRINT_CMD
    Printf("CmdReadFormatCapacities\r");
#endif
    ReadFormatCapacitiesResponse.Length = 0x08;
    ReadFormatCapacitiesResponse.NumberOfBlocks = __REV(MsdMem::BlockCnt);
    // 01b Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b Formatted Media - Current media capacity
    // 11b No Cartridge in Drive - Maximum formattable capacity
    ReadFormatCapacitiesResponse.DescCode = 0x02;
    ReadFormatCapacitiesResponse.BlockSize[0] = (uint8_t)(MsdMem::BlockSz >> 16);
    ReadFormatCapacitiesResponse.BlockSize[1] = (uint8_t)(MsdMem::BlockSz >> 8);
    ReadFormatCapacitiesResponse.BlockSize[2] = (uint8_t)(MsdMem::BlockSz);
    // Transmit Data
    TransmitBuf((uint32_t*)&ReadFormatCapacitiesResponse, sizeof(ReadFormatCapacitiesResponse));
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= sizeof(ReadFormatCapacitiesResponse);
    return retv::Ok;
}

struct AddrLen_t { uint32_t Addr, Len; };

StatusOr<AddrLen_t> PrepareAddrAndLen() {
    StatusOr<AddrLen_t> r;
    r->Addr = Convert::BuildU132(CmdBlock.SCSICmdData[5], CmdBlock.SCSICmdData[4], CmdBlock.SCSICmdData[3], CmdBlock.SCSICmdData[2]);
    r->Len  = Convert::BuildU16(CmdBlock.SCSICmdData[8], CmdBlock.SCSICmdData[7]);
    // Check block addr
    if((r->Addr + r->Len) > MsdMem::BlockCnt) {
        Printf("Out Of Range: Addr %u, Len %u\r", r->Addr, r->Len);
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        r.Rslt = retv::Fail;
    }
    // Check cases 4, 5: (Hi != Dn); and 3, 11, 13: (Hn, Ho != Do)
    if(CmdBlock.DataTransferLen != r->Len * MsdMem::BlockSz) {
        Printf("Wrong length\r");
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
        r.Rslt = retv::Fail;
    }
    return r; // Ok by default
}

retv CmdRead10() {
#if DBG_PRINT_CMD
    Printf("CmdRead10\r");
#endif
    StatusOr<AddrLen_t> al = PrepareAddrAndLen();
    if(!al.Ok()) return retv::Fail;
    // Send data
    uint32_t BlocksToRead, BytesToSend; // Intermediate values
    while(al->Len != 0) {
        BlocksToRead = MIN_(MSD_DATABUF_SZ / MsdMem::BlockSz, al->Len);
        BytesToSend = BlocksToRead * MsdMem::BlockSz;
        if(MsdMem::Read(al->Addr, (uint8_t*)Buf32, BlocksToRead) == retv::Ok) {
            TransmitBuf(Buf32, BytesToSend);
            CmdBlock.DataTransferLen -= BytesToSend;
            al->Len  -= BlocksToRead;
            al->Addr += BlocksToRead;
        }
        else {
            Printf("Rd fail\r");
            // TODO: handle read error
            return retv::Fail;
        }
    } // while
    return retv::Ok;
}

retv CmdWrite10() {
#if DBG_PRINT_CMD
    Printf("CmdWrite10\r");
#endif
#if MSD_READ_ONLY
    SenseData.SenseKey = SCSI_SENSE_KEY_DATA_PROTECT;
    SenseData.AdditionalSenseCode = SCSI_ASENSE_WRITE_PROTECTED;
    SenseData.AdditionalSenseQualifier = SCSI_ASENSEQ_NO_QUALIFIER;
    return retvFail;
#else
    // Check case 8: Hi != Do
    if(CmdBlock.Flags & 0x80) {
        SenseData.SenseKey = SCSI_SENSE_KEY_ILLEGAL_REQUEST;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_INVALID_COMMAND;
        return retv::Fail;
    }
    // TODO: Check if ready
    if(false) {
        SenseData.SenseKey = SCSI_SENSE_KEY_NOT_READY;
        SenseData.AdditionalSenseCode = SCSI_ASENSE_MEDIUM_NOT_PRESENT;
        return retv::Fail;
    }
    // Get transaction size
    StatusOr<AddrLen_t> al = PrepareAddrAndLen();
    if(!al.Ok()) return retv::Fail;
//    Printf("Addr=%u; Len=%u\r", BlockAddress, TotalBlocks);
    uint32_t BlocksToWrite, BytesToReceive;
    retv Rslt = retv::Ok;

    while(al->Len != 0) {
        // Fill Buf
        BytesToReceive = MIN_(MSD_DATABUF_SZ, al->Len * MsdMem::BlockSz);
        BlocksToWrite  = BytesToReceive / MsdMem::BlockSz;
        if(ReceiveToBuf(Buf32, BytesToReceive) != retv::Ok) {
            Printf("Rcv fail\r");
            return retv::Fail;
        }
        // Write Buf to memory
        Rslt = MsdMem::Write(al->Addr, (uint8_t*)Buf32, BlocksToWrite);
        if(Rslt != retv::Ok) return retv::Fail;
        CmdBlock.DataTransferLen -= BytesToReceive;
        al->Len -= BlocksToWrite;
        al->Addr += BlocksToWrite;
    } // while
    return retv::Ok;
#endif
}

retv CmdModeSense6() {
#if DBG_PRINT_CMD
    Printf("CmdModeSense6\r");
#endif
    uint16_t RequestedLength = CmdBlock.SCSICmdData[4];
    uint16_t BytesToTransfer = MIN_(RequestedLength, MODE_SENSE6_DATA_SZ);
    TransmitBuf((uint32_t*)&Mode_Sense6_data, BytesToTransfer);
    // Succeed the command and update the bytes transferred counter
    CmdBlock.DataTransferLen -= BytesToTransfer;
    return retv::Ok;
}
#endif
