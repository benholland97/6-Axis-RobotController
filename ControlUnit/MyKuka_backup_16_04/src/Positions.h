/*
 * Positions.h
 *
 *  Created on: 12 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_POSITIONS_H_
#define SRC_POSITIONS_H_

#include "Utils.h"
#include "Config.h"
#include <vector>
#include <utility>

class MyPoint {
public:
    MyPoint();

    MyPoint(const MyPoint& p);

    MyPoint(double* coords):x(coords[0]),y(coords[1]),z(coords[2]){};

    MyPoint(double pX, double pY, double pZ):x(pX),y(pY),z(pZ){};

    void set(double pX, double pY, double pZ);

    void null();

    bool isNull();

	void operator= (const Vector& p) {
		x = p[0];
		y = p[1];
		z = p[2];
	}

    double& operator[] (int idx) {
        switch(idx) {
            case 0: return x; break;
            case 1: return y; break;
            case 2: return z; break;
            default: return x; break;
        }
    }

    double operator[] (int idx) const {
        switch(idx) {
            case 0: return x; break;
            case 1: return y; break;
            case 2: return z; break;
            default: return x; break;
        }
    }

    double compare(MyPoint rhs);

    void fixAlmostZero();

    friend ostream& operator<<(ostream& os, const MyPoint& p);

//private:
    double x,y,z;
};

class Rotation: public MyPoint {
public:
	Rotation () : MyPoint (0,0,0){};
	//Roll, Pitch, Yaw
	Rotation(double pX,double pY, double pZ): MyPoint(pX,pY,pZ) {
		x = pX;
		y = pY;
		z = pZ;
	}

	Rotation(const Rotation& r) : MyPoint(r) {
		x= r.x;
		y= r.y;
		z= r.z;
	};

	friend ostream& operator<<(ostream& os, const Rotation& r);

};


class JointAngles {
public:
    JointAngles(){
        null();
    }

    JointAngles(double *pAngles) {
        for (int i = 0;i<NO_ACTUATORS; ++i)
			a[i] = toRadians(pAngles[i]);
    }

    JointAngles(const JointAngles& pJA) {
		for (int i = 0;i<NO_ACTUATORS;++i)
			a[i] = pJA.a[i];
	}


    void setAnglesRad(double *pA) {
        for (int i = 0;i<NO_ACTUATORS;++i)
			a[i] = pA[i];
    }

    void operator = (const JointAngles& pJA){
		for (int i = 0;i<NO_ACTUATORS;++i)
			a[i] = pJA.a[i];
    }

    double* getAngles() {
    	return a;
    }

    void setDefaultPosition() {
        a[0] = 0.0;
        a[1] = 0.0;
        a[2] = 0.0;
        a[3] = 0.0;
        a[4] = 0.0;
        a[5] = 0.0;
//        a[6] = 0.0;
    }

    static JointAngles getDefaultPosition() {
        JointAngles ja;
        ja.setDefaultPosition();
        return ja;
    };

    double& operator[](int idx) {
		if ((idx >= 0) || ( idx < NO_ACTUATORS))
			return a[idx];
        static double dummy(0);
		return dummy;
	}

	const double& operator[](int idx) const {
		if ((idx >= 0) || ( idx < NO_ACTUATORS))
			return a[idx];
        static double dummy(0);
		return dummy;
	}

    void null() {
		for (int i = 0;i<NO_ACTUATORS;++i)
			a[i] = 0.0;
	}

	bool isNull() {
		for (int i = 0;i<NO_ACTUATORS;++i)
			if (a[i] != 0.0)
				return false;
		return true;
	}

	bool limitsCheck();

	double distanceBetween(JointAngles rhs);

	void fixAlmostZero();

    friend ostream& operator<<(ostream& os, const JointAngles& ja);
//private:
    double a[NO_ACTUATORS];
};




class FullPosition {
public:
    FullPosition() {
        null();
    }

    FullPosition(const FullPosition& pIP): FullPosition() {
        position = pIP.position;
        orientation = pIP.orientation;
        angles = pIP.angles;
    }

    FullPosition(const MyPoint& pPosition, const Rotation& pOrientation) {
        position = pPosition;
        orientation = pOrientation;
        angles.null();
    };

    FullPosition(const JointAngles& pAngles) {
        position.null();
        orientation.null();
        angles = pAngles;
    };

    void null() {
        orientation.null();
        position.null();
        angles.null();
    }

    bool isNull() {
        return position.isNull();
    }

    bool anglesSet() {
        return angles.isNull();
    }

    void fixAlmostZero();

    double compare(FullPosition rhs);

    friend ostream& operator<<(ostream& os, const FullPosition& fp);

// private:
    MyPoint position;
    Rotation orientation;
    JointAngles angles;
};

#endif /* SRC_POSITIONS_H_ */
