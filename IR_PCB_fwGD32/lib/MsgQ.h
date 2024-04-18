/*
 * EvtMsg.h
 *
 *  Created on: 21 ���. 2017 �.
 *      Author: Kreyl
 */

#ifndef MSGQ_H_
#define MSGQ_H_

#include <inttypes.h>
//#include "gd_lib.h"
#include "EvtMsgIDs.h"
#include "yartos.h"
//#include "board.h"
#if BUTTONS_ENABLED
#include "buttons.h"
#endif

/*
 * Example of msg:
 * union RMsg_t {
    uint32_t DWord[3];
    CmdUniversal_t Cmd;
    RMsg_t& operator = (const RMsg_t &Right) {
        DWord[0] = Right.DWord[0];
        DWord[1] = Right.DWord[1];
        DWord[2] = Right.DWord[2];
        return *this;
    }
    RMsg_t() {
        DWord[0] = 0;
        DWord[1] = 0;
        DWord[2] = 0;
    }
    RMsg_t(CmdUniversal_t *PCmd) {
        Cmd.CmdID = PCmd->CmdID;
        Cmd.SnsID = PCmd->SnsID;
        Cmd.w16[0] = PCmd->w16[0];
        Cmd.w16[1] = PCmd->w16[1];
        Cmd.w16[2] = PCmd->w16[2];
    }
} __attribute__((__packed__));
[...]
EvtMsgQ_t<RMsg_t, RMSG_Q_LEN> MsgQ;
*/

#define MAIN_EVT_Q_LEN      11  // Messages in queue
#define EMSG_DATA8_CNT      7   // ID + 7 bytes = 8 = 2x DWord32
#define EMSG_DATA16_CNT     3   // ID + 3x2bytes = 3

union EvtMsg_t {
    uint32_t dword[2];
    struct {
        union {
            void* Ptr;
            struct {
                int32_t Value;
                uint8_t ValueID;
            } __attribute__((__packed__));
//            uint8_t b[EMSG_DATA8_CNT];
//            uint16_t w16[EMSG_DATA16_CNT];
#if BUTTONS_ENABLED
            BtnEvtInfo_t BtnEvtInfo;
#endif
        } __attribute__((__packed__));
        EvtId ID;
    } __attribute__((__packed__));

    EvtMsg_t& operator = (const EvtMsg_t &Right) {
        dword[0] = Right.dword[0];
        dword[1] = Right.dword[1];
        return *this;
    }
    EvtMsg_t() : Ptr(nullptr), ID(EvtId::None) {}
    EvtMsg_t(EvtId AID) : Ptr(nullptr), ID(AID) {}
    EvtMsg_t(EvtId AID, void *APtr) : Ptr(APtr), ID(AID) {}
    EvtMsg_t(EvtId AID, int32_t AValue) : Value(AValue), ID(AID) {}
    EvtMsg_t(EvtId AID, uint8_t AValueID, int32_t AValue) : Value(AValue), ValueID(AValueID), ID(AID) {}
} __attribute__((__packed__));


template<typename T, uint32_t sz>
class EvtMsgQ_t {
private:
    union {
        uint64_t __Align;
        T IBuf[sz];
    };
    T *read_ptr, *write_ptr;
    Semaphore_t full_sem;    // Full slots counter
    Semaphore_t empty_sem;   // Empty slots counter
public:
    EvtMsgQ_t() : __Align(0), read_ptr(IBuf), write_ptr(IBuf) {}
    void Init() {
        read_ptr = IBuf;
        write_ptr = IBuf;
        empty_sem.Init(sz);
        full_sem.Init(0);
    }

    /* Retrieves a message from a mailbox, returns empty Msg if failed.
     * The invoking thread waits until a message is posted in the mailbox
     * for a timeout (may be TIME_INFINITE or TIME_IMMEDIATE) */
    T Fetch(systime_t Timeout) {
        T Msg;
        Sys::Lock();
        if(full_sem.WaitTimeoutS(Timeout) == retv::Ok) {
            // There is something in the queue, get it
            Msg = *read_ptr++;
            if(read_ptr >= &IBuf[sz]) read_ptr = IBuf;  // Circulate pointer
            empty_sem.SignalI();
            Sys::RescheduleS();
        }
        Sys::Unlock();
        return Msg;
    }

    /* Post a message into a mailbox.
     * The function returns a timeout condition if the queue is full */
    retv SendNowOrExitI(const T &Msg) {
        if(empty_sem.GetCntI() <= 0L) return retv::Timeout; // Q is full
        empty_sem.FastWaitI();
        *write_ptr++ = Msg;
        if(write_ptr >= &IBuf[sz]) write_ptr = IBuf;  // Circulate pointer
        full_sem.SignalI();
        return retv::Ok;
    }

    retv SendNowOrExit(const T &Msg) {
        Sys::Lock();
        retv Rslt = SendNowOrExitI(Msg);
        Sys::RescheduleS();
        Sys::Unlock();
        return Rslt;
    }

    /* Posts a message into a mailbox.
     * The invoking thread waits until a empty slot in the mailbox becomes available
     * or the specified time runs out. */
    retv SendWaitingAbility(const T &Msg, systime_t timeout) {
        Sys::Lock();
        retv msg = empty_sem.WaitTimeoutS(timeout);
        if(msg == retv::Ok) {
            *write_ptr++ = Msg;
            if(write_ptr >= &IBuf[sz]) write_ptr = IBuf;  // Circulate pointer
            full_sem.SignalI();
            Sys::RescheduleS();
        }
        Sys::Unlock();
        if(msg == retv::Timeout) return retv::Timeout;
        else if(msg == retv::Ok) return retv::Ok;
        else return retv::Fail;
    }

    uint32_t GetFullCnt() {
        Sys::Lock();
        uint32_t rslt = full_sem.GetCntI();
        Sys::Unlock();
        return rslt;
    }
};

/* Always presents in main:
 * EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
 * ...
 * EvtQMain.Init();
 * ...
 * EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
 * switch(Msg.ID) {
        case evtIdButtons:
        break;
        ...
        default: Printf("Unhandled Msg %u\r", Msg.ID); break;
    } // Switch
 */
extern EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;

#endif // MSGQ_H_
