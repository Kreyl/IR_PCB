/*
 * ir_pkt.h
 *
 *  Created on: 13.08.2023
 *      Author: layst
 */

#ifndef IR_PKT_H_
#define IR_PKT_H_

#include "types.h"
#include "shell.h"

int32_t Damage_IdToHits(int32_t damage_id);
StatusOrI32 Damage_HitsToId(int32_t hits);

enum class PktType {
    Shot = 0x0000,
    AddHealth = 0x8000,
    AddCartridges = 0x8100,
    NewGame = 0x8305,
};

#pragma pack(push, 1)
union IRPkt {
    uint16_t word16;
    uint8_t bytes[2];
    struct { // LSB to MSB
        uint32_t _reserved: 2; // Not used in 14-bit pkt
        uint32_t damage_id: 4;
        uint32_t team_id: 2;
        uint32_t player_id: 7;
        uint32_t zero: 1;
    };

    IRPkt() : word16(0) {}
    IRPkt(uint16_t AW16) : word16(AW16) {}

    void PrintI() { PrintfI("Pkt: Word %04X; PlayerID %u; TeamID %u; DamageID %d\r", word16, player_id, team_id, damage_id); }
    void Print() {
        Sys::Lock();
        PrintI();
        Sys::Unlock();
    }

    int32_t GetDamageHits() { return Damage_IdToHits(damage_id); }

    IRPkt& operator =(const IRPkt &Right) { word16 = Right.word16; return *this; }
};
#pragma pack(pop)

#endif /* IR_PKT_H_ */
