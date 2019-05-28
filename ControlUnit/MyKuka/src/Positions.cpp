/*
 * Positions.cpp
 *
 *  Created on: 12 Mar 2019
 *      Author: Ben
 */

#include "Positions.h"


MyPoint::MyPoint() {
    null();
}

MyPoint::MyPoint(const MyPoint& p) {
	x = p[0];
	y = p[1];
	z = p[2];
}

void MyPoint::null() {
	x = 0.0;
	y = 0.0;
	z = 0.0;
};

bool MyPoint::isNull() {
	return 	(almostEqual(x,0.0,FLOAT_PRECISION) && almostEqual(y,0.0,FLOAT_PRECISION) && almostEqual(z,0.0,FLOAT_PRECISION));
};

void MyPoint::set(double pX, double pY,double pZ) {
	x = pX;
	y = pY;
	z = pZ;
}

double MyPoint::compare(MyPoint rhs) {
	double res(0);
	res+= fabs(x) - fabs(rhs.x);
	res+= fabs(y) - fabs(rhs.y);
	res+= fabs(z) - fabs(rhs.z);
	return res;
}


void MyPoint::fixAlmostZero() {
	almostZeroFix(x,FLOAT_PRECISION);
	almostZeroFix(y,FLOAT_PRECISION);
	almostZeroFix(z,FLOAT_PRECISION);
}

void JointAngles::fixAlmostZero() {
	for(int i=0; i<NO_ACTUATORS; ++i) {
		almostZeroFix(a[i],FLOAT_PRECISION);
	}
}

bool JointAngles::limitsCheck() {
	double offsets[NO_ACTUATORS-1] = {ANGLE0_OFFSET,ANGLE1_OFFSET,ANGLE2_OFFSET,ANGLE3_OFFSET,ANGLE4_OFFSET,ANGLE5_OFFSET};
	toRadians(offsets,NO_ACTUATORS-1);
	double maxLower = toRadians(MG996R_MAX_ANGLE);
	double maxUpper = toRadians(MG90S_MAX_ANGLE);
	for(int i=0; i<NO_MG996R; ++i) {
		if(a[i] > (maxLower+offsets[i]) || a[i] < (-maxLower+offsets[i])) {
			return false;
		}
	}
	for(int i=NO_MG996R; i<NO_ACTUATORS; ++i) {
		if(a[i] > (maxUpper+offsets[i]) || a[i] < (-maxUpper+offsets[i])) {
			return false;
		}
	}
	return true;
}

double JointAngles::distanceBetween(JointAngles rhs) {
	double sum;
	for(int i=0; i<NO_ACTUATORS-1; ++i) {
		sum += fabs(a[i]-rhs.a[i]);
	}
	return sum;
}

void FullPosition::fixAlmostZero() {
	position.fixAlmostZero();
	orientation.fixAlmostZero();
	angles.fixAlmostZero();
}

double FullPosition::compare(FullPosition rhs) {
	double diff(0);
	diff = position.compare(rhs.position) + orientation.compare(rhs.orientation);
	return diff;
}



ostream& operator<<(ostream& os, const MyPoint& p) {
    os << setprecision(5)<<"X: " <<p[0] << "\tY: " << p[1] << "\tZ: "<< p[2] <<" ";
    return os;
}


ostream& operator<<(ostream& os, const Rotation& r) {
    os << setprecision(5)<<"R: " <<toDegrees(r[0]) << "\tP: " << toDegrees(r[1]) << "\tY: "<< toDegrees(r[2]) <<" ";
    return os;
}

ostream& operator<<(ostream& os, const JointAngles& ja) {
    os << setprecision(5)<<"{ 0: " <<toDegrees(ja[0]) << "\t 1: " << toDegrees(ja[1]) << "\t 2: "<< toDegrees(ja[2]) <<"\t 3: "
    		<<toDegrees(ja[3])<<"\t 4: "<<toDegrees(ja[4])<<"\t 5: "<<toDegrees(ja[5])<<"\t 6: "<<toDegrees(ja[6])<<" } ";
    return os;
}

ostream& operator<<(ostream& os, const FullPosition& fp) {
	FullPosition tFP = fp;
	tFP.fixAlmostZero();
	os<<"\nPosition: "<<tFP.position <<"\nOrientation: "<<tFP.orientation<<"\nJoint Angles: "<<tFP.angles<<"\n";
	return os;
}





