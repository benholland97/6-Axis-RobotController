/*
 * Config.h
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_


//----------HARDWARE CONSTANTS (mm)---------------
#define NO_ACTUATORS	6
//#define MG90S_MAX_ANGLE 180
#define MG90S_MAX_ANGLE 90
#define MG90S_MIN_ANGLE -90


//#define HIP_HEIGHT  	100
#define HIP_HEIGHT  	135
//#define SHOULDER_OFFSET  35
#define SHOULDER_OFFSET  0
//#define HUMERUS_LENGTH  120
#define HUMERUS_LENGTH  135
//#define FOREARM_LENGTH  125
#define FOREARM_LENGTH  145
// #define WRIST_LENGTH    55 //w/o hand
//#define WRIST_LENGTH    135 //w hand
#define WRIST_LENGTH    0 //w hand


//----------SOFTWARE CONSTANTS (mm)---------------
#define FLOAT_PRECISION 0.000000001
#define EPSILON         0.1

//------OPENCV-------
#define HSV_MIN_VAL		100
#define HSV_MAX_VAL		255
#define HUE_RED_CNTR	175
#define HUE_RED_RNGE	15

#define ARUCO_DICT		3 	//DICT_4X4_1000
#define MARKER_LENGTH	40	//mm
#define ARUCO_XOFFSET	-100
#define ARUCO_YOFFSET	0


#define BLOCK_WIDTH 	24
#define BLOCK_LENGTH	75
#define BLOCK_DEPTH		15
#define BLOCK_TOLERANCE 20








#endif /* SRC_CONFIG_H_ */
