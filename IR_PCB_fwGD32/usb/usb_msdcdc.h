/*
 * usb_mst.h
 *
 *  Created on: 2016/01/29 �.
 *      Author: Kreyl
 */

#ifndef USB_MSD_CDC_H__
#define USB_MSD_CDC_H__

#include "shell.h"

#if 1 // ================= Mass Storage constants and types ====================
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
#endif

#define MSD_TIMEOUT_MS   2700
#define MSD_DATABUF_SZ   4096

class UsbMsdCdc_t : public Shell_t {
private:
    void IStartTransmissionIfNotYet(); // Required for printf implementation
    retv IPutChar(char c);             // Required for printf implementation
public:
    void Init();
    void Reset();
    void Connect();
    void Disconnect();
    bool IsActive();
    retv TryParseRxBuff(); // Call this when something ip received
    retv ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
    retv TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms);
};

extern UsbMsdCdc_t UsbMsdCdc;

#endif // USB_MSD_CDC_H__
