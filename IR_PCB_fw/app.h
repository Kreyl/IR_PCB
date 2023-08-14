/*
 * app.h
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: layst
 */

#ifndef APP_H_
#define APP_H_

#include <inttypes.h>

enum class FirEvt { Reset, StartFire, EndOfFiring };


class Firing_h {

public:

};

extern Firing_h Firing;

namespace Controls {

bool DoBurstFire();

}

#endif /* APP_H_ */
