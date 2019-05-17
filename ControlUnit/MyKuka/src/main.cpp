/*
 * main.cpp
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#include "Kinematics.h"
#include "MovementController.h"
#include "ImageProcessor.h"
#include <stdio.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <array>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


int main (int argc, char* argv[]) {
	cout<<"Stating robot arm controller\n";

//	MovementController::enumeratePorts();
	MovementController mc;

	if(mc.isReady()) {
		cout <<"Serial connection established - Ready for action"<<endl;
		this_thread::sleep_for(chrono::milliseconds(3000));
		mc.init();
	}
	else {
		cout<<"Failed to establish serial connection";
		return 0;
	}
	mc.flushSerial();

#ifdef ARM_CALIBRATION
	string s = "res/calibTargets.txt";
	ifstream f(s.c_str());
	if(!f.good()) mc.generateCalibTargetFile(s);
#endif
	ImageProcessor ip(2);

	ip.userAction(mc);

	cout<<mc.readSerial();

	cout<<"\nIts over";

	return 1;
}




