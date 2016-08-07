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
        if (almostEqual(ca,0))
            ca = 0;
        if (almostEqual(ca,1))
            ca = 1.0;
        if (almostEqual(ca,-1))
            ca = -1.0;

        sa = sin(val_alpha);
        if (almostEqual(sa,0))
            sa = 0;
        if (almostEqual(sa,1))
            sa = 1.0;
        if (almostEqual(sa,-1))
            sa = -1.0;

};


