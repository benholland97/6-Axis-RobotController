/*
 * main.cpp
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "Kinematics.h"
#include "SerialPort.h"
#include "MovementController.h"
#include "ImageProcessor.h"
#include <stdio.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
//#include "serial/serial.h";

using namespace std;
using namespace cv;

//void usrInputIK(Kinematics& kin, double* a) {
//	double input[6] = {0,0,0,0,0,0};
//	cout<<"\nEnter pose in format Position, Orientation \n";
//    string s;
//    vector<double> result;
//    getline(cin,s);
//    stringstream ss(s);
//    while(ss.good()) {
//        string substr;
//        getline( ss, substr, ',' );
//        result.push_back(stod(substr));
//    }
//    for(int i=0; i<6; ++i) {
//    	input[i] = result[i];
//    }
//	FullPosition fp1 = FullPosition(MyPoint(input[0],input[1],input[2]),Rotation(toRadians(input[3]),toRadians(input[4]),toRadians(input[5])));
////	cout<<"Pre Calc Pos"<<fp1;
//	kin.calcIK(fp1);
//	fp1.fixAlmostZero();
//	cout<<"Post Calc Pos"<<fp1;
//	for(int i=0; i<6; ++i) {
//		a[i] = toDegrees(fp1.angles.a[i]);
//	}
//}

//int sendOverSerial(bool usrInput = false) {
//	cout<<"Initiating serial comms\n";
//
//	char *port = "\\\\.\\COM4";
//
//	SerialPort* SP = new SerialPort(port);    // adjust as needed
//
//	char incomingData[255];
//
//	if(SP->isConnected()) {
//		cout<<"Connection established\n";
//		SP->readSerialPort(incomingData, MAX_DATA_LENGTH);
//	} else {
//		cout<<"ERROR, check ports\n";
////		this_thread::sleep_for(chrono::milliseconds(200));
////		return 0;
//	}
////	int dataLength = 255;
//	bool setMoveFlag = true;
//	Kinematics kin;
//
//	while(SP->isConnected()) {
//		memset(incomingData, 0, 255);
//		double angles[6] = {0,0,0,0,0,0};
//		int step = 5;
//		int size = 360/step;
//		double z[size],y[size];
//
//		if(usrInput) {
//			usrInputIK(kin,angles);
//		} else {
//			setMoveCircle(220,0,5,5,z,y);
//		}
//		int idx = 0;
//		while(setMoveFlag) {
//			if(!usrInput) {
//				FullPosition fp = FullPosition(MyPoint(250,y[idx],z[idx]),Rotation(0,0,0));
////				kin.calcIK(fp);
//				for(int i=0; i<NO_ACTUATORS;++i) {
//					angles[i] = toDegrees(fp.angles.a[i]);
//				}
//				idx++;
//				cout<<"Looping through circle :"<<idx<<"\n";
//				sleep(500);
//				if(idx >= size) return 0;
//			}
//			string s;
//	//		stringstream ss;
//			s += "<";
//			for(int i=0; i<NO_ACTUATORS-1; ++i) {
//				s += to_string_with_precision(angles[i]) + ",";
//			}
//			s+=to_string_with_precision<double>(angles[NO_ACTUATORS-1]) + ">";
//	//		string s = ss.str();
//		    char *c_string = new char[s.size()+1];
//		    copy(s.begin(),s.end(), c_string);
//		    c_string[s.size()] = '\n';
//
//		    cout<<"Sending :"<<c_string;
//		    int res = SP->writeSerialPort(c_string,s.size()+1);
//		    this_thread::sleep_for(chrono::milliseconds(200));
//		    cout<<"Result of send: "<<res<<".\n";
//
//	//	    readResult = SP->readSerialPort(incomingData, MAX_DATA_LENGTH);
//	//	    cout<<incomingData;
//		    int n = 0;
//		    while(SP->readSerialPort(incomingData, MAX_DATA_LENGTH)) {
//	//	    	cout<<"loop :"<<n<<"\n";
//	//	    	n++;
//		    	cout<<incomingData;
//		    }
//		    delete[] c_string;
//
//		    if(usrInput) setMoveFlag = false;
//		}
//
//	}
//	return 0;
//}



int sendSerialLinux(){
	char data[] = {0,0,0,0,0,0};
	FILE *file;
    if( NULL == (file = fopen("/dev/ttyACM0","w") ) )  //Opening device file
    { // then fopen failed
        perror("fopen failed for ttyACM0" );
        exit( EXIT_FAILURE );
    }

}

int main (int argc, char* argv[]) {
	cout<<"Stating robot arm controller\n";
	ImageProcessor ip(2);
//	= ImageProcessor(2);

	cout<<"Ready for block location? "<<ip.ready();

	ip.userAction();

	cout<<"Ready for block location? "<<ip.ready();



//    this_thread::sleep_for(chrono::milliseconds(3000));
//	Kinematics kin;
//	Serial serial("COM4");
//    MovementController mc("COM4");
//	FullPosition fp = FullPosition(MyPoint(280,0,0),Rotation(0,0,0));

//	kin.calcIK(fp);

//	cout<<"NEW fp \n"<<fp;

//	while(!mc.isReady()) {
//		cout<<"Attempting to establish connection\n";
//		mc.setSerialPort("COM4");
//	}
//
//	while(1) {
//		mc.setMoveCircle(250,0,200,20,10);
//		this_thread::sleep_for(chrono::milliseconds(3000));
////		break;
//	}
	cout<<"\nIts over";

	return 1;
}




