/*
 * EvtMsgIDs.h
 *
 *  Created on: 21 ���. 2017 �.
 *      Author: Kreyl
 */

#ifndef EVTMSGIDS_H__
#define EVTMSGIDS_H__

enum class EvtId : uint8_t {
    None = 0, // Always

    UartCheckTime,
    EverySecond,

    // Usb
    UsbConnect,
    UsbDisconnect,
    UsbReady,
    UsbDataRcvd,
};

#endif //EVTMSGIDS_H__
