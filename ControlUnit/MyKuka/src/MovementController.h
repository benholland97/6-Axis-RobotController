/*
 * MovementController.h
 *
 *  Created on: 30 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_MOVEMENTCONTROLLER_H_
#define SRC_MOVEMENTCONTROLLER_H_

#include "Kinematics.h"
#include "Config.h"
#include <serial.h>
#include "Utils.h"
#include <cstddef>
#include <iostream>
#include <fstream>
#include <array>
#include <algorithm>
#include <ctime>
#include <cstdlib>

using namespace serial;

#define SEND_DELAY_MS	1000

class MovementController {
public:

	MovementController(Kinematics _k, string path = DEF_ARD_PTH):kin(_k),serialPort(path,BAUD_RATE,Timeout::simpleTimeout(1000)),sendFile("res/transmissionLog.txt"){};

	MovementController(Kinematics _k):kin(_k),sendFile("res/transmissionLog.txt"){};

	MovementController(string path=DEF_ARD_PTH):kin(Kinematics()),serialPort(path,BAUD_RATE,Timeout::simpleTimeout(1000)),sendFile("res/transmissionLog.txt"){};

	bool isReady(){
		return serialPort.isOpen();
	}

	void init() {
		moveHome();
	}

	void setSerialPort(string path);

	void setSerialBaud(int baud);

//	void setSerialTimeout(int timeout);

	void setMoveCircle(int c1, int c2, int c3, int rad, int step, int plane = 0);

//	string readAllSerial();

	static void enumeratePorts();

	void flushSerial() {
		string s;
		while(serialPort.read(s)>0);
	}

	string readSerial();

	bool setTargetPosition(FullPosition targetPos, bool gripperClosed=false);

	bool moveHome();

	bool initBlockGrab(FullPosition pos, double angle);

	void generateCalibTargetFile(string fileName="res/calibTargets.txt");


private:
	vector<FullPosition> angleJumpPosFiller(JointAngles target, JointAngles current, int pos);
	bool angleJumpCheck(JointAngles target, JointAngles current, int& idx);
	vector<FullPosition> setTrajectorySimple(FullPosition target, FullPosition current);
	vector<FullPosition> setTrajectoryAngles(FullPosition target, FullPosition current, bool gripperClosed=false);
	double calcBezierValue(double a, double supportA, double b, double supportB, double t);
	bool sendPosition(FullPosition data);
	vector<int>  sendPositions(vector<FullPosition> _data);
	vector<FullPosition> getPositionsOnLine3d(FullPosition l1, FullPosition l2, int num=TARGET_ITERS);
	FullPosition currentPos;
	void calcCircleCoords(int cx, int cy, int radius, int step, double *_y, double *_z);
	bool sendResult();
	bool placeOperation();


//	bool sendResultTimeout();

//	bool configFlag;
	Kinematics kin;
	Serial serialPort;
	ofstream sendFile;

};

#endif /* SRC_MOVEMENTCONTROLLER_H_ */
