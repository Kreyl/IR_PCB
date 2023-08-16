/*
 * ir_pkt.h
 *
 *  Created on: 13 авг. 2023 г.
 *      Author: layst
 */

#ifndef IR_PKT_H_
#define IR_PKT_H_

#include <inttypes.h>
#include "shell.h"

#define PKT_TYPE_RESET  0
#define PKT_TYPE_SHOT   1

union IRPkt_t {
    uint16_t W16;
    struct {
        uint32_t Type: 3;
        uint32_t FightID: 2;
        uint32_t TeamID: 3;
        uint32_t PktN: 5;
        uint32_t crc: 3;
    };
    void CalculateCRC() {
        uint16_t Sum = 0;
        for(int i=2; i<=14; i+=2) {
            uint16_t w = W16 << i;
            Sum ^= w;
        }
        crc = (Sum >>= 14) & 0b111U;
    }
    bool IsCrcOk() {
        uint32_t OldCrc = crc;
        CalculateCRC();
        return OldCrc == crc;
    }
    void Print() { Printf("Pkt: Word %04X; Type %u; FightID %u; TeamID %u; PktN %u; crc %u\r",
            W16, Type, FightID, TeamID, PktN, crc); }
    IRPkt_t& operator =(const IRPkt_t &Right) { W16 = Right.W16; return *this; }
};

#endif /* IR_PKT_H_ */
