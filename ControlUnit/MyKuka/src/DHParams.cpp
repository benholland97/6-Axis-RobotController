/*
 * DHParams.cpp
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#include "DHParams.h"

ostream& operator<<(ostream& os, const DHParam& dh) {
    os << setprecision(2) << "Alpha: " <<dh._alpha << "\tR: " << dh._r << "\tD: "<< dh._d <<" ";
    return os;
}

DHParam::DHParam() {
	init(0,0,0);
};

DHParam::DHParam(const double  pAlpha, const double pR, const double pD) {
	init(pAlpha, pR, pD);
};

// initialize with the passed Denavit Hardenberg params and precompute sin/cos
void DHParam::init(const double pAlpha, const double pR, const double pD) {
		_r = pR;
		_d = pD;
		_alpha = pAlpha;

        ca = cos(_alpha);
        sa = sin(_alpha);
};

