/*
 * Utils.h
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#include <cmatrix>
#include <cmath>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <fstream>


#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


//#define PI 			3.1415926535897932384626433832795028841971693993751058209749445923078164062
#define PI			3.14159265358979323846
#define PI_2		1.57079632679489661923

using namespace std;


using techsoft::mslice;
typedef techsoft::matrix<double>  Matrix;
typedef valarray<double> Vector;


void mySleep(unsigned long milliseconds);
double toRadians(double degrees);
void toRadians(double* degArr, int size);
double toDegrees(double radians);
double* toDegrees(double* radArr, int size);
bool almostEqual(double a, double b, double precision);
bool almostZeroFix(double& a, double precision);
void almostZeroFix(double* arr, double length, double precision);
void almostZeroFix(Matrix& m, double precision);
double hypotenuseLength(double a, double b);
double distanceBetweenTwoPoints(double ax, double ay, double bx, double by);
double distanceBetweenTwoPoints(double ax, double ay, double az, double bx, double by, double bz);
char* strToCharArr(string s);
bool inRange(double a, double lower, double upper);
int withinBlockDimens(double blockWidth, double blockHeight, double blockDepth, double width, double height, double tolerance);
bool inRange2(double a, double target, double tol);
bool withinSphereCheck(double cx, double cy, double cz, double x, double y, double z, double r);

template <typename T>
string to_string_with_precision(const T a_value, const int n = 2)
{
    ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}



#endif /* SRC_UTILS_H_ */
