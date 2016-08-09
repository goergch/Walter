/*
 * DenavitHardenbergParam.cpp
 *
 *  Created on: 07.08.2016
 *      Author: SuperJochenAlt
 */

#include <DenavitHardenbergParam.h>


void DenavitHardenbergParams::init(const rational pAlpha, const rational pA, const rational pD) {
		val_a = pA;
		val_d = pD;
		val_alpha = pAlpha;

        ca = cos(val_alpha);
        sa = sin(val_alpha);
};


