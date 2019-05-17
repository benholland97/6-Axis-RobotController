///*
// * SerialPort.h
// *
// *  Created on: 12 Mar 2019
// *      Author: Ben
// */
//
//#ifndef SERIALPORT_H
//#define SERIALPORT_H
//
//#define ARDUINO_WAIT_TIME 2000
//#define MAX_DATA_LENGTH 255
//#define TIMEOUT_DELAY 200
//
////#include <windows.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string>
//#include <cstring>
////#include <stdio.h>
//#include <iostream>
//#include "Utils.h"
//
//
//
//using namespace std;
//
//class SerialPort
//{
//private:
//    HANDLE handler;
//    bool connected;
//    COMSTAT status;
//    DWORD errors;
//public:
//    SerialPort(char *portName);
//    ~SerialPort();
//
//    int readSerialPort(char *buffer, unsigned int buf_size);
//    bool writeSerialPort(char *buffer, unsigned int buf_size);
//    bool isConnected();
//};
//
//class Serial {
//public:
//	Serial(){};
//
//	Serial(string pName);
//
//	Serial(const Serial& _s):serialPort(_s.serialPort){};
//
//	bool setPort(string pName);
//
//	bool setPort(string pName, float timeoutMS);
//
//
//	~Serial();
//
//
//	bool isConnected() {
//		return serialPort->isConnected();
//	}
//
//	bool write(string sData);
//
//	//returns empty string if no message
//	string read();
//
//private:
//	SerialPort* serialPort;
//};
//
//#endif // SERIALPORT_H
