#ifndef CONFIG_H
#define CONFIG_H

#include <ArduinoSTL.h>

#define M_PI			3.14159265358979323846

//--------------PINS------------------

//------SERVOS--------
#define NO_ACTUATORS  7

#define MG996R_MAX_ANGLE 160
#define MG996R_MAX_ANGLE_2 MG996R_MAX_ANGLE/2
#define MG90S_MAX_ANGLE 180
#define MG90S_MAX_ANGLE_2 MG90S_MAX_ANGLE/2
#define MG90S_MIN_ANGLE 0
#define MG996R_MIN_ANGLE 0

//Hip
#define SERVO0_OFFSET   0
// #define SERVO0_OFFSET   -15
#define SERVO0MIN  95 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO0MAX  484

//Waist
#define SERVO1_OFFSET   60
// #define SERVO1_OFFSET   50
#define SERVO1MIN  95 
#define SERVO1MAX  380


//Shoulder
#define SERVO2_OFFSET   -3
// #define SERVO2_OFFSET   0
#define SERVO2MIN  95 
#define SERVO2MAX  460

//Elbow
#define MG90S_ELB_MIN_ANGLE -45
#define MG90S_ELB_MAX_ANGLE 45
#define SERVO3_OFFSET   -16
// #define SERVO3_OFFSET   0
// #define SERVO3MIN  165 
// #define SERVO3MAX  375
#define SERVO3MIN  95 
#define SERVO3MAX  460

//Wrist
#define SERVO4_OFFSET   15
#define SERVO4MIN  95 
#define SERVO4MAX  440

//Gripper-rotation
// #define SERVO5_OFFSET   -20
#define SERVO5_OFFSET   7
#define SERVO5MIN  95 
#define SERVO5MAX  484

//Gripper
#define SERVO6_OFFSET   0
#define SERVO6MIN  95 
#define SERVO6MAX  484
#define GRIPPER_OPEN    305
#define GRIPPER_CLOSE    435
// #define GRIPPER_CLOSE    415


//------RXEIVER--------
#define EI_ARDUINO_INTERRUPTED_PIN
#define NUM_RX_CHANNELS  6
#define RX_OFFSET   2
#define RX_AVG      1500

#define PIN_RX0     2
#define RX0_MIN  1148
#define RX0_MAX  1888

#define PIN_RX1     3
#define RX1_MIN  1136
#define RX1_MAX  1796

#define PIN_RX2     4
#define RX2_MIN  1156
#define RX2_MAX  1816

#define PIN_RX3     6
#define RX3_MIN  1160
#define RX3_MAX  1908

#define PIN_RX4     6
#define RX4_MIN  996
#define RX4_MAX  2012

#define PIN_RX5   7
#define RX5_MIN   996
#define RX5_MAX  2016


//------SERIAL_RX--------
#define NUM_CHARS   64


//------MATRIX VALUES-----------
#define NUM_MATRIX_ROWS    4
#define NUM_MATRIX_COLUMNS    4
#define N              4

//----------HARDWARE CONSTANTS (mm)---------------
#define HIP_HEIGHT  100
#define SHOULDER_OFFSET  35
#define HUMERUS_LENGTH  120
#define FOREARM_LENGTH  100
// #define WRIST_LENGTH    55 //w/o hand
#define WRIST_LENGTH    115 //w hand


//----------SOFTWARE CONSTANTS (mm)---------------
#define FLOAT_PRECISION 0.00001
#define EPSILON         0.2

#endif