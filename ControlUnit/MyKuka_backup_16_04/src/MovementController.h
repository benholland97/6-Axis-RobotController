///*
// * MovementController.h
// *
// *  Created on: 30 Mar 2019
// *      Author: Ben
// */
//
//#ifndef SRC_MOVEMENTCONTROLLER_H_
//#define SRC_MOVEMENTCONTROLLER_H_
//
//#include "Kinematics.h"
//#include "SerialPort.h"
//#include <algorithm>
//
//#define SEND_DELAY_MS	1000
//
//class MovementController {
//public:
//	MovementController(Kinematics _k, Serial _s):kin(_k),serial(_s){
//		configFlag = serial.isConnected();
//	};
//
//	MovementController(string pName);
//
//	bool isReady(){
//		return configFlag;
//	}
//
//	bool setSerialPort(string pName);
//
//	void setMoveCircle(int c1, int c2, int c3, int rad, int step, int plane = 0);
//
//private:
//	void sendPositions(vector<FullPosition> _data);
//	void calcCircleCoords(int cx, int cy, int radius, int step, double *_y, double *_z);
//	bool sendResult();
//	bool sendResultTimeout();
//
//
//	bool configFlag;
//	Kinematics kin;
//	Serial serial;
//};
//
//#endif /* SRC_MOVEMENTCONTROLLER_H_ */
