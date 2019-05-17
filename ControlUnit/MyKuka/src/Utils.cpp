/*
 * Utils.cpp
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#include "Utils.h"


void mySleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}


double toRadians(double degrees) {
	return (degrees * PI) / 180;
//	const double fac = (PI/ 180.0);
//	return degrees * fac;
}

void toRadians(double* degArr, int size) {
	for(int i=0; i<size; ++i) {
		degArr[i] = toRadians(degArr[i]);
	}
}

double  toDegrees(double radians) {
	return (radians * 180) / PI;
//	const double fac = (180.0/ PI);
//	return radians * fac;
}

double* toDegrees(double* radArr, int size) {
	for(int i=0; i<size; ++i) {
		radArr[i] = toDegrees(radArr[i]);
	}
	return radArr;
}

bool almostEqual(double a, double b, double precision) {
	if (a==b)
		return true;
	if (a == 0)
		return (fabs(b)<precision);
	if (b == 0)
		return (fabs(a)<precision);

	if (b<a)
		return (fabs((b/a)-1.0) < precision);
	else
		return (fabs((a/b)-1.0) < precision);
}

bool almostZeroFix(double& a, double precision) {
	if(fabs(a) < precision) {
		a = 0;
		return true;
	}
	return false;
}

void almostZeroFix(Matrix& m, double precision) {
	for(int i=0; i<(int)m.rowno(); ++i) {
		for(int j=0; j<(int)m.colno(); ++j) {
			double d = m[i][j];
			m[i][j] = almostZeroFix(d,precision)?0:d;
		}
	}
}

double hypotenuseLength(double a, double b) {
    return sqrt(a*a+b*b);
}


double distanceBetween(double* a, double* b, int size) {
	double sum;
	for(int i=0; i<size; ++i) {
		sum += fabs(a[i]-b[i]);
	}
	return sum;
}

double distanceBetweenTwoPoints(double ax, double ay, double bx, double by){
	return sqrt(((ax-bx)*(ax-bx)) + ((ay-by)*(ay-by)));
}

double distanceBetweenTwoPoints(double ax, double ay, double az, double bx, double by, double bz){
	return sqrt(((ax-bx)*(ax-bx)) + ((ay-by)*(ay-by))+ ((az-bz)*(az-bz)));
}


char* strToCharArr(string s){
	char* c = new char[s.size() + 1];
	strcpy(c,s.c_str());
	return c;
}

bool inRange(double a, double lower, double upper) {
	return a > upper ? false : a < lower? false : true;
}

bool inRange2(double a, double target, double tol) {
	if(fabs(a-target) < tol) {
		return true;
	}
	return false;
}


int withinBlockDimens(double blockWidth, double blockHeight, double blockDepth, double _width, double _height, double tolerance){
	double width = min(_width, _height);
	double height = max(_width, _height);

//	//largest face up
//	if(inRange(width,blockWidth-tolerance,blockWidth+tolerance)
//			&& inRange(height,blockHeight-tolerance,blockHeight+tolerance)) {
//		return 0;
//	}
//	//side up
//	if(inRange(width,blockDepth-tolerance,blockDepth+tolerance)
//			&& inRange(height,blockHeight-tolerance,blockHeight+tolerance)) {
//		return 1;
//	}
//	//end up
//	if(inRange(width,blockDepth-tolerance,blockDepth+tolerance)
//			&& inRange(height,blockWidth-tolerance,blockWidth+tolerance)) {
//		return 2;
//	}

	if(inRange2(width,blockWidth,tolerance) && inRange2(height,blockHeight,tolerance)) return 0;
	if(inRange2(width,blockDepth,tolerance) && inRange2(height,blockHeight,tolerance)) return 1;
//	if(inRange2(width,blockDepth,tolerance) && inRange2(height,blockWidth,tolerance)) return 2;

	return -1;
}



bool withinSphereCheck(double cx, double cy, double cz, double x, double y, double z, double r){
	double dist = distanceBetweenTwoPoints(x,y,z,cx,cy,cz);
	//Point lies within sphere
	return dist < r*r? true:false;
}





