/*
 * Config.h
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_


//----------HARDWARE CONSTANTS (mm)---------------
#define NO_ACTUATORS	7
//#define MG90S_MAX_ANGLE 180
#define MG90S_MAX_ANGLE 90
#define MG90S_MIN_ANGLE -90
#define MG996R_MAX_ANGLE 80
#define MG996R_MIN_ANGLE -80

#define NO_MG996R	3
#define NO_MG90S	4

#define HIP_HEIGHT  	100
#define SHOULDER_OFFSET  35
#define HUMERUS_LENGTH  120
#define FOREARM_LENGTH  125
// #define WRIST_LENGTH    65 //w/o hand
//#define WRIST_LENGTH    135 //w hand
#define WRIST_LENGTH    105 //w hand

#define REST_X_POS		295
#define REST_Y_POS		0
#define REST_Z_POS		220

#define BLK_GRB_HEIGHT	150

#define HOME_X_POS		85
#define HOME_Y_POS		0
#define HOME_Z_POS		210

#define ANGLE0_OFFSET	0
#define ANGLE1_OFFSET	60
#define ANGLE2_OFFSET	-3
#define ANGLE3_OFFSET	-16
#define ANGLE4_OFFSET	15
#define ANGLE5_OFFSET	7




//----------SOFTWARE CONSTANTS (mm)---------------
#define FLOAT_PRECISION 0.000000001
#define EPSILON         0.1

//------OPENCV-------
#define HSV_MIN_VAL		100
#define HSV_MAX_VAL		255
#define HUE_RED_CNTR	175
#define HUE_RED_RNGE	15
#define CANNY_THRESH	175

#define ARUCO_DICT		3 	//DICT_4X4_1000
#define MARKER_LENGTH	40	//mm
#define ARUCO_XOFFSET	100
#define ARUCO_YOFFSET	0
#define ARUCO_BORDER	3


#define BLOCK_WIDTH 	24
#define BLOCK_LENGTH	75
#define BLOCK_DEPTH		15
#define BLOCK_TOLERANCE 30

#define TRGT_MV_DLY		10000

//#define TRGT_MOVE

//------SERIAL-------
#define DEF_ARD_PTH		"/dev/ttyUSB0"
#define BAUD_RATE		9600
#define SERIAL_TIMEOUT	1000

//------TRAJECTORY-------
#define TARGET_ITERS	30
#define TGT_ANGLE_THRESH	20

//------CALIBRATION-------
//#define ARM_CALIBRATION
//#define OPENCV_CALIB
#define NO_POINTS		5
#define NO_ITERS		20
#define ARM_RADIUS		300






#endif /* SRC_CONFIG_H_ */
