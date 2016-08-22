/*
 * DenavitHardenbergParam.h
 *
 *  Created on: 07.08.2016
 *      Author: JochenAlt
 */

#ifndef DENAVITHARDENBERGPARAM_H_
#define DENAVITHARDENBERGPARAM_H_

#include "setup.h"
#include "Util.h"

// Class that represents Denavit-Hardenberg parameters.
class DenavitHardenbergParams{
public:
	DenavitHardenbergParams() {
		init(0,0,0);
	};
	DenavitHardenbergParams(const rational  pAlpha, const rational pA, const rational pD) {
		init(pAlpha, pA, pD);
	};

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
