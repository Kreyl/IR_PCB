/*
 * kl_buf.h
 *
 *  Created on: 07.04.2013
 *      Author: kreyl
 */

#ifndef KL_BUF_H_
#define KL_BUF_H_

#include <inttypes.h>
#include "types.h"
//#include "string.h" // for memcpy

// Simple buffer
struct Buf_t {
    uint32_t Sz = 0;
    uint8_t *Ptr = nullptr;
};

template <typename T>
struct BufType_t {
    uint32_t Sz = 0;
    T* Ptr = nullptr;
};

template <uint32_t MaxSz>
struct BufSz_t {
    uint32_t Sz;
    uint8_t Buf[MaxSz];
};

template <typename T, uint32_t MaxSz>
struct BufTypeSz_t {
    uint32_t Sz;
    T Buf[MaxSz];
};

#if 1 // ============================== Circular ===============================
template <typename T, uint32_t Sz>
class CircBuf_t {
private:
    T IBuf[Sz], *PRead=IBuf, *PWrite=IBuf;
    uint32_t FullCnt = 0;
public:
    inline uint32_t GetEmptyCount() { return Sz - FullCnt; }
    inline uint32_t GetFullCount()  { return FullCnt; }
    inline bool IsEmpty() { return (FullCnt == 0); }
    inline void Flush() {
        PRead = IBuf;
        PWrite = IBuf;
        FullCnt = 0;
    }

    // Clear N elements
    void Flush(uint32_t N) {
        if(N > FullCnt) N = FullCnt;
        FullCnt -= N;
        PRead += N;
        if(PRead >= (IBuf + Sz)) PRead -= Sz;
    }

    uint32_t GetMany(T *p, uint32_t ALength) {
        uint32_t Cnt = 0;
        while(FullCnt != 0 and Cnt < ALength) {
            *p++ = *PRead++;
            if(PRead >= (IBuf + Sz)) PRead = IBuf;
            Cnt++;
            FullCnt--;
        }
        return Cnt; // return how many items were written
    }

    Buf_t GetContinuousChunkAndDoNotRemove() {
        Buf_t rb;
        rb.Ptr = PRead;
        if(PRead < PWrite) rb.Sz = PWrite - PRead;
        else if(PRead >= PWrite and FullCnt != 0) rb.Sz = (IBuf + Sz) - PRead; // Data from PRead to right bound
        return rb;
    }

    retv PutMany(T *p, uint32_t Length) {
        if(GetEmptyCount() >= Length) {    // check if enough space
            uint32_t PartSz = (IBuf + Sz) - PWrite;  // Data from PWrite to right bound
            if(Length > PartSz) {
                memcpy(PWrite, p, PartSz * sizeof(T));
                PWrite = IBuf;     // Start from beginning
                p += PartSz;
                Length -= PartSz;
            }
            memcpy(PWrite, p, Length * sizeof(T));
            PWrite += Length;
            if(PWrite >= (IBuf + Sz)) PWrite = IBuf; // Circulate pointer
            FullCnt += Length;
            return retv::Ok;
        }
        else return retv::Overflow;
    }

    retv Get(T *p) {
        if(IsEmpty()) return retv::Empty;
        else {
            *p = *PRead;
            PRead++;
            if(PRead > (IBuf + Sz - 1)) PRead = IBuf; // Circulate buffer
            FullCnt--;
            return retv::Ok;
        }
    }

    retv GetAndDoNotRemove(T *p) {
        if(IsEmpty()) return retv::Empty;
        *p = *PRead;
        return retv::Ok;
    }

    // Put anyway
    void PutI(T Value) {
        *PWrite = Value;
        PWrite++;
        if(PWrite > (IBuf + Sz - 1)) PWrite = IBuf; // Circulate buffer
        FullCnt++;
    }

    retv PutIfNotOverflow(T Value) {
        if(FullCnt < Sz) {
            *PWrite++ = Value;
            if(PWrite > (IBuf + Sz - 1)) PWrite = IBuf; // Circulate buffer
            FullCnt++;
            return retv::Ok;
        }
        else return retv::Overflow;
    }

    retv PutPIfNotOverflow(T *PValue) {
        if(FullCnt < Sz) { // Leave at least one slot
            *PWrite = *PValue;
            PWrite++;
            if(PWrite > (IBuf + Sz - 1)) PWrite = IBuf;   // Circulate buffer
            FullCnt++;
            return retv::Ok;
        }
        else return retv::Overflow;
    }
};
#endif

#if 0 // =================== Circular buffer with memcpy =======================
template <typename T, uint32_t Sz>
class CircBufMemcpy_t {
protected:
    uint32_t IFullSlotsCount=0;
    T IBuf[Sz], *PRead=IBuf, *PWrite=IBuf;
public:
    uint8_t Get(T *p) {
        if(IFullSlotsCount == 0) return retvEmpty;
        memcpy(p, PRead, sizeof(T));
        if(++PRead > (IBuf + Sz - 1)) PRead = IBuf;     // Circulate buffer
        IFullSlotsCount--;
        return retvOk;
    }
    uint8_t GetPAndMove(T **pp) {
    	if(IFullSlotsCount == 0) return retvEmpty;
    	*pp = PRead;
        if(++PRead > (IBuf + Sz - 1)) PRead = IBuf;     // Circulate buffer
        IFullSlotsCount--;
        return retvOk;
    }
    uint8_t GetLastP(T **pp) {
    	if(IFullSlotsCount == 0) return retvEmpty;
		*pp = PRead;
		return retvOk;
    }

    uint8_t PutAnyway(T *p) {
		memcpy(PWrite, p, sizeof(T));
		if(++PWrite > (IBuf + Sz - 1)) PWrite = IBuf;   // Circulate buffer
		if(IFullSlotsCount < Sz) IFullSlotsCount++;
		return retvOk;
	}
    uint8_t Put(T *p) {
        if(IFullSlotsCount >= Sz) return retvOverflow;
        return PutAnyway(p);
    }

    inline bool IsEmpty() { return (IFullSlotsCount == 0); }
    inline uint32_t GetEmptyCount() { return Sz-IFullSlotsCount; }
    inline uint32_t GetFullCount()  { return IFullSlotsCount; }
    void Flush(uint32_t ALength) {
        LimitMaxValue(ALength, IFullSlotsCount);
        IFullSlotsCount -= ALength;
        uint32_t PartSz = (IBuf + Sz) - PRead;
        if(ALength >= PartSz) {
            ALength -= PartSz;
            PRead = IBuf + ALength;
        }
        else PRead += ALength;
    }
    void Flush() {
        IFullSlotsCount = 0;
        PRead = PWrite;
    }
};
#endif

#if 0 // =============== Buffer operating with pointers ========================
template <typename T, uint32_t Sz>
class CircPtrBuf_t {
protected:
    uint32_t IFullSlotsCount=0;
    T IBuf[Sz], *PRead=IBuf, *PWrite=IBuf;
public:
    // Returns pointer to valid object or nullptr if empty
    T* GetReadPtr() { return (IFullSlotsCount == 0)? nullptr : PRead; }

    void MoveReadPtr() {
        if(++PRead > (IBuf + Sz - 1)) PRead = IBuf;     // Circulate buffer
        if(IFullSlotsCount) IFullSlotsCount--;
    }

    T* GetWritePtr() { return (IFullSlotsCount >= Sz)? nullptr : PWrite; }
    void MoveWritePtr() {
        if(++PWrite > (IBuf + Sz - 1)) PWrite = IBuf;   // Circulate buffer
        if(IFullSlotsCount < Sz) IFullSlotsCount++;
    }

    inline bool IsEmpty() { return (IFullSlotsCount == 0); }
    inline uint32_t GetEmptyCount() { return Sz-IFullSlotsCount; }
    inline uint32_t GetFullCount()  { return IFullSlotsCount; }

    void Flush() {
        IFullSlotsCount = 0;
        PRead = PWrite;
    }
};
#endif

#if 1 // =========================== Many buffers ==============================
// Append to buf0, until filled up. Switch buffers and return addrSwitch.
template <typename T, uint32_t Sz, uint32_t BufCnt>
class BufQ_t {
private:
    T Buf[BufCnt][Sz];
    uint32_t BufIndxW = 0, BufIndxR = 0;
    uint32_t FullBufCnt= 0, FullSlotsCnt = 0;
public:
    bool IsEmpty() { return (FullBufCnt == 0) and (FullSlotsCnt == 0); }
    bool IsFullBufPresent() { return FullBufCnt != 0; }

    // Put if not overflow
    retv Put(T Value) {
        if(FullBufCnt >= BufCnt and FullSlotsCnt >= Sz) return retv::Overflow;
        Buf[BufIndxW][FullSlotsCnt++] = Value;
        if(FullSlotsCnt >= Sz) {
            FullSlotsCnt = 0;
            BufIndxW++;
            if(BufIndxW >= BufCnt) BufIndxW = 0;
            FullBufCnt++;
        }
        return retv::Ok;
    }

    // Returns last full or partially full buffer. Does not mark it as empty.
    BufType_t<T> GetAndLockBuf() {
        BufType_t<T> r;
        if(FullBufCnt > 0) {
            r.Ptr = &Buf[BufIndxR][0];
            r.Sz = Sz;
        }
        else if(FullSlotsCnt > 0) {
            r.Ptr = &Buf[BufIndxR][0];
            r.Sz = FullSlotsCnt;
            // Do not use the buf, write to next one
            BufIndxW++;
            if(BufIndxW >= BufCnt) BufIndxW = 0;
            FullSlotsCnt = 0;
            FullBufCnt++;
        }
        return r;
    }

    void UnlockBuf() {
        if(FullBufCnt > 0) {
            BufIndxR++;
            if(BufIndxR >= BufCnt) BufIndxR = 0;
            FullBufCnt--;
        }
    }
};
#endif

#if 1 // ========================= Counting buffer =============================
template <typename T, uint32_t Sz>
class CountingBuf_t {
private:
    T IBuf[Sz];
    uint32_t Cnt;
public:
    void Add(T Value) {
        for(uint32_t i=0; i<Cnt; i++) {
            if(IBuf[i] == Value) return;   // do not add what exists
        }
        IBuf[Cnt] = Value;
        Cnt++;
    }
    uint32_t GetCount() { return Cnt; }
    void Clear() { Cnt = 0; }
};
#endif

#if 0 // ============================ LIFO =====================================
template <typename T, uint32_t Sz>
class LifoNumber_t {
protected:
    uint32_t Cnt=0;
    T IBuf[Sz];
public:
    uint8_t Put(T Value) {
        if(Cnt == Sz) return retvOverflow;
        IBuf[Cnt] = Value;
        Cnt++;
        return retvOk;
    }

    uint8_t Get(T *p) {
        if(Cnt == 0) return retvEmpty;
        Cnt--;
        *p = IBuf[Cnt];
        return retvOk;
    }

    uint8_t GetAndDoNotRemove(T *p) {
        if(Cnt == 0) return retvEmpty;
        *p = IBuf[Cnt-1];
        return retvOk;
    }

    inline uint32_t GetFullCount()  { return Cnt; }
};

#endif

#if 0 // ================= LIFO buffer operating with pointers =================
template <typename T, int32_t Sz>
class LifoPtrBuf_t {
private:
    int32_t Indx=-1;
    T IBuf[Sz];
public:
    T* GetPtr() { return (Indx >= 0)? &IBuf[Indx] : nullptr; }
    uint8_t Push() {
        if(Indx < Sz) {
            Indx++;
            return retvOk;
        }
        else return retvOverflow;
    }
    void Pop()  { if(Indx >= 0) Indx--; }
    void Flush() { Indx = -1; }
};
#endif

#if 0 // ================= Static Storage with validity ========================
template <typename T, uint32_t Sz>
class StorageWValidity_t {
private:
    T IBuf[Sz];
    bool IsValid[Sz];
    uint32_t Cnt = 0;

    uint32_t GetValidByIndx(uint32_t Indx) {
        uint32_t FIndx = 0;
        for(uint32_t i=0; i<Sz; i++) {
            if(IsValid[i]) {
                if(FIndx == Indx) return i;
                else FIndx++;
            }
        }
        return 0;
    }
public:
    T* Add() {
        if(Cnt >= Sz) return nullptr;
        else {
            // Find empty slot: iterate all valid ones
            uint32_t FIndx = 0;
            while(IsValid[FIndx]) FIndx++;
            IsValid[FIndx] = true; // Validate
            Cnt++;
            return &IBuf[FIndx];
        }
    }

    T* operator[](const uint32_t Indx) {
        if(Indx >= Cnt) return nullptr;
        else return &IBuf[GetValidByIndx(Indx)];
    }

    void Remove(const uint32_t Indx) {
        if(Indx < Cnt) {
            int32_t FIndx = GetValidByIndx(Indx);
            if(IsValid[FIndx]) {
                IsValid[FIndx] = false;
                Cnt--;
            }
        }
    }

    void Remove(T* ptr) {
        for(uint32_t i=0; i<Sz; i++) {
            if(ptr == &IBuf[i]) {
                if(IsValid[i]) {
                    IsValid[i] = false;
                    Cnt--;
                }
                return;
            }
        } // for
    }

    void RemoveAll() {
        for(uint32_t i=0; i<Sz; i++) IsValid[i] = false;
        Cnt = 0;
    }

    int32_t GetCnt() { return Cnt; }
};
#endif

#if 0 // ========================= String list =================================
class Stringlist_t {
private:
    uint32_t AllocatedCnt = 0;
    uint32_t MaxCnt = 0xFFFFFFFF;
public:
    uint32_t Cnt = 0;
    char **Strings = nullptr;

    uint8_t AddAndCopyString(const char* S) {
        // Append array of pointers if needed
        if(Cnt == AllocatedCnt) {
            if(AllocateCnt(Cnt + 1) != retvOk) return retvOutOfMemory;
        }
        // Copy string
        uint32_t Len = strlen(S);
        if(Len > 0) {
            Strings[Cnt] = (char*)malloc(Len + 1);
            if(!Strings[Cnt]) return retvFail;
            strcpy(Strings[Cnt], S);
            Cnt++;
        }
        return retvOk;
    }

    uint8_t AllocateCnt(uint32_t ACnt) {
        if(ACnt > MaxCnt) return retvOverflow;
        char **tmp = (char**)realloc(Strings, ACnt * sizeof(char*));
        if(tmp) {
            Strings = tmp;
            AllocatedCnt = ACnt;
            return retvOk;
        }
        else return retvOutOfMemory;
    }

    // Delete all strings, delete array of pointers
    void Clear() {
        if(Strings) {
            for(uint32_t i=0; i<Cnt; i++) if(Strings[i]) free(Strings[i]);
            free(Strings);
            Cnt = 0;
            Strings = nullptr;
        }
    }

    char* operator[](const int32_t Indx) { return Strings[Indx]; }

    Stringlist_t() {}
    Stringlist_t(uint32_t AMaxCnt) : MaxCnt(AMaxCnt) {}

    ~Stringlist_t() {
        if(Strings) {
            for(uint32_t i=0; i<Cnt; i++) if(Strings[i]) free(Strings[i]);
            free(Strings);
        }
    }
};
#endif

#if 0 // ============================ ID List ==================================
template <typename T, uint32_t Sz>
class IdList_t {
private:
    T IBuf[Sz];
public:
    uint32_t Cnt = 0, CurrIndx = 0;

    void Clear() {
        Cnt = 0;
        CurrIndx = 0;
    }

    uint8_t Add(T AValue) {
        if(Cnt < Sz) {
            IBuf[Cnt++] = AValue;
            return retvOk;
        }
        else return retvOverflow;
    }

    uint8_t AddIfNotYet(T AValue) {
        for(T IValue : IBuf) {
            if(IValue == AValue) return retvOk; // already in
        }
        return Add(AValue);
    }

    void Remove(T AValue) {
        uint32_t FIndx = 0;
        while(true) {
            if(FIndx >= Cnt) return; // Not found
            if(IBuf[FIndx] == AValue) break;
            FIndx++;
        }
        // Found, replace it
        while((FIndx + 1) < Cnt) {
            IBuf[FIndx] = IBuf[FIndx+1];
            FIndx++;
        }
        Cnt--;
        if(CurrIndx >= Cnt) CurrIndx = Cnt-1;
    }

    T GetCurrent() { return IBuf[CurrIndx]; }
    void MoveToNext() {
        CurrIndx++;
        if(CurrIndx >= Cnt) CurrIndx = 0;
    }

    T operator[](const uint32_t Indx) { return IBuf[Indx]; }
};
#endif

#endif // KL_BUF_H_
