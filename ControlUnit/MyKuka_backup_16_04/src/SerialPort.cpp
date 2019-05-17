///*
// * SerialPort.cpp
// *
// *  Created on: 12 Mar 2019
// *      Author: Ben
// */
//#include "SerialPort.h"
//
//SerialPort::SerialPort(char *portName)
//{
//    this->connected = false;
//
//    this->handler = CreateFileA(static_cast<LPCSTR>(portName),
//                                GENERIC_READ | GENERIC_WRITE,
//                                0,
//                                NULL,
//                                OPEN_EXISTING,
//                                FILE_ATTRIBUTE_NORMAL,
//                                NULL);
//    if (this->handler == INVALID_HANDLE_VALUE){
//        if (GetLastError() == ERROR_FILE_NOT_FOUND){
//            printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
//        }
//    else
//        {
//            printf("ERROR!!!");
//        }
//    }
//    else {
//        DCB dcbSerialParameters = {0};
//
//        if (!GetCommState(this->handler, &dcbSerialParameters)) {
//            printf("failed to get current serial parameters");
//        }
//        else {
//            dcbSerialParameters.BaudRate = CBR_9600;
//            dcbSerialParameters.ByteSize = 8;
//            dcbSerialParameters.StopBits = ONESTOPBIT;
//            dcbSerialParameters.Parity = NOPARITY;
//            dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;
//
//            if (!SetCommState(handler, &dcbSerialParameters))
//            {
//                printf("ALERT: could not set Serial port parameters\n");
//            }
//            else {
//                this->connected = true;
//                PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
//                Sleep(ARDUINO_WAIT_TIME);
//            }
//        }
//    }
//}
//
//SerialPort::~SerialPort()
//{
//    if (this->connected){
//        this->connected = false;
//        CloseHandle(this->handler);
//    }
//}
//
//int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
//{
//    DWORD bytesRead;
//    unsigned int toRead = 0;
//
//    ClearCommError(this->handler, &this->errors, &this->status);
//
//    if (this->status.cbInQue > 0){
//        if (this->status.cbInQue > buf_size){
//            toRead = buf_size;
//        }
//        else toRead = this->status.cbInQue;
//    }
//
//    memset(buffer, 0, buf_size);
//
//    if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL)) return bytesRead;
//
//    return 0;
//}
//
//bool SerialPort::writeSerialPort(char *buffer, unsigned int buf_size)
//{
//    DWORD bytesSend;
//
//    if (!WriteFile(this->handler, (void*) buffer, buf_size, &bytesSend, 0)){
//        ClearCommError(this->handler, &this->errors, &this->status);
//        return false;
//    }
//    else return true;
//}
//
//bool SerialPort::isConnected()
//{
//    if (!ClearCommError(this->handler, &this->errors, &this->status))
//		this->connected = false;
//
//    return this->connected;
//}
//
//Serial::Serial(string pName) {
//	setPort(pName);
//}
//
//
//Serial::~Serial() {
//	delete serialPort;
//}
//
//bool Serial::setPort(string pName, float timeoutMS) {
//	cout<<"Establishing serial connection\n";
//	for(int i=0; i<(int)timeoutMS/(int)TIMEOUT_DELAY; ++i){
//		cout<<"Attempt No : \t"<<i;
//		if(setPort(pName)) return true;
//	    this_thread::sleep_for(chrono::milliseconds(TIMEOUT_DELAY));
//	}
//	return false;
//}
//
//bool Serial::setPort(string pName) {
//	if(pName.at(0) != '\\') {
//		pName = "\\\\.\\"+pName;
//	}
//	cout<<"TRYING to attach to port "<<strToCharArr(pName)<<"\n";
//	serialPort = new SerialPort(strToCharArr(pName));
//
//	if(serialPort->isConnected()) {
//		cout<<"Connection established\n";
//		return 1;
//	} else {
//		cout<<"ERROR, check ports\n";
//		return 0;
//	}
//}
//
//
//
//bool Serial::write(string sData){
//	char cData[sData.size() + 1];
//	strcpy(cData,sData.c_str());
//	return serialPort->writeSerialPort(cData,sData.size()+1);
//}
//
//string Serial::read(){
//	char retBuf[MAX_DATA_LENGTH];
//	string retStr = "";
//	while(serialPort->readSerialPort(retBuf, MAX_DATA_LENGTH)) {
//		retStr += retBuf;
//		memset(retBuf, 0, MAX_DATA_LENGTH);
//	}
//	return retStr;
//}
//
//
//
//
//
