/*
 * DHParams.h
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_DHPARAMS_H_
#define SRC_DHPARAMS_H_

#include "Utils.h"

class DHParam{
public:
	DHParam();

	DHParam(const double  pAlpha, const double pR, const double pD);

	void init(const double pAlpha, const double pR, const double pD);

	double getR() const { return _r; };

	const double getD() const { return _d; };

	const double getAlpha() const { return _alpha; };

	const double sinalpha() const { return sa; };

	const double cosalpha() const { return ca; };

    friend ostream& operator<<(ostream& os, const DHParam& dh);


private:
	double _r;
	double _d;
	double _alpha;
	double ca;
	double sa;
};

#endif /* SRC_DHPARAMS_H_ */
