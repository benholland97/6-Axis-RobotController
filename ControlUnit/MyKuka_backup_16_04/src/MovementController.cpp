///*
// * MovementController.cpp
// *
// *  Created on: 30 Mar 2019
// *      Author: Ben
// */
//
//#include "MovementController.h"
//
//MovementController::MovementController(string pName){
//	configFlag = serial.setPort(pName,2000);
//}
//
//
//bool MovementController::setSerialPort(string pName){
//	configFlag = serial.setPort(pName);
//	return configFlag;
//}
//
//
//void MovementController::setMoveCircle(int c1, int c2, int c3, int rad, int step, int plane){
//	cout<<"Set move - circle \n";
//	int size = 360/step;
//	double a1[size], a2[size];
//	switch(plane) {
//	case 1:
//		calcCircleCoords(c1,c3,rad,step,a1,a2);
//		break;
//	case 2:
//		calcCircleCoords(c1,c2,rad,step,a1,a2);
//		break;
//	default:
//		calcCircleCoords(c2,c3,rad,step,a1,a2);
//		break;
//	}
//
//	vector<FullPosition> sendVec;
//	for(int i=0; i<size; ++i) {
//		MyPoint p;
//		Rotation r(0,0,0);
//		switch(plane) {
//		case 1:
//			p.set((double)a1[i],(double)c3,(double)a2[i]);
//			break;
//		case 2:
//			p.set((double)a1[i],(double)a2[i],(double)c3);
//			break;
//		default:
//			p.set((double)c3,(double)a1[i],(double)a2[i]);
//			break;
//		}
//		FullPosition fp = FullPosition(p,r);
//		kin.calcIK(fp);
//		sendVec.push_back(fp);
//	}
////	cout<<"Sending vector\n";
////	for(vector<FullPosition>::iterator it = sendVec.begin(); it != sendVec.end(); ++it) {
////		cout<<*it<<"\n";
////	}
//
//	sendPositions(sendVec);
//}
//
//void MovementController::sendPositions(vector<FullPosition> data){
//	for(int i=0; i<(int)data.size(); ++i) {
//		string sendStr = "<";
//		for(int j=0; j<NO_ACTUATORS-1; ++j) {
//			sendStr += to_string_with_precision(toDegrees(data[i].angles[j])) + ",";
//		}
//		sendStr+= to_string_with_precision<double>(toDegrees(data[i].angles[NO_ACTUATORS - 1])) + ">";
//		cout<<"Attempting to send \t"<<sendStr<<"\n";
//		if(serial.write(sendStr)) {
//			sendResultTimeout();
//		}
//	}
//}
//
//bool MovementController::sendResultTimeout() {
//	chrono::steady_clock::time_MyPoint start = chrono::steady_clock::now();
//	while(!sendResult()) {
//		if(chrono::steady_clock::now() - start > chrono::milliseconds(SEND_DELAY_MS)) {
//			cout<<"Send result timed out \n";
//			break;
//		}
//	}
//	return true;
//
//}
//
//bool MovementController::sendResult(){
//	string buffer = serial.read();
//	cout<<buffer;
//	if (buffer.find("<1>") != std::string::npos) {
//	    return true;
//	}
//	return false;
//}
//
//void MovementController::calcCircleCoords(int c1, int c2, int radius, int step, double *_a1, double *_a2){
//	for(int i=0; i<360; i+=step) {
//		double angle = toRadians(i);
//		_a1[i/step] = c1 + (radius * cos(angle));
//		_a2[i/step] = c2 + (radius * sin(angle));
//	}
//}
//
//
