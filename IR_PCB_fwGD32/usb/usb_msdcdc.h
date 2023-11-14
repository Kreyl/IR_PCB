/*
 * usb_mst.h
 *
 *  Created on: 2016/01/29 �.
 *      Author: Kreyl
 */

#ifndef USB_MSD_CDC_H__
#define USB_MSD_CDC_H__

#include "shell.h"

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
