/*
 * DenavitHardenbergParam.cpp
 *
 * Precomputed DenavitHardenberg parameters are required by forward kinematics
 * Author: JochenAlt
 */

#ifndef DENAVITHARDENBERGPARAM_H_
#define DENAVITHARDENBERGPARAM_H_

#include "setup.h"
#include "Util.h"

class DenavitHardenbergParams{
public:
	DenavitHardenbergParams();
	DenavitHardenbergParams(const rational  pAlpha, const rational pA, const rational pD);
	void init(const rational pAlpha, const rational pA, const rational pD);
	rational getA() const { return val_a; };
	const rational getD() const { return val_d; };
	const rational getAlpha() const { return val_alpha; };

	const rational sinalpha() const { return sa; };
	const rational cosalpha() const { return ca; };

private:
	rational val_a;
	rational val_d;
	rational val_alpha;
	rational ca;
	rational sa;
};

#endif /* DENAVITHARDENBERGPARAM_H_ */
