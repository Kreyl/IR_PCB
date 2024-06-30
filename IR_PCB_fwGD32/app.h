/*
 * app.h
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: layst
 */

#ifndef APP_H_
#define APP_H_

#include <inttypes.h>

extern int32_t hit_cnt, rounds_cnt, magazines_cnt;

void AppInit();
void SetInputs(uint32_t AIn[2]);
void Reset(bool quiet = false);

void IrRxCallbackI(uint32_t Rcvd);

#endif /* APP_H_ */
