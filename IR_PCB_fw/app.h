/*
 * app.h
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: layst
 */

#ifndef APP_H_
#define APP_H_

#include <inttypes.h>
#include "MsgQ.h"


namespace Firing {



void Init();



}


namespace Controls {

bool DoBurstFire();

}

#endif /* APP_H_ */
