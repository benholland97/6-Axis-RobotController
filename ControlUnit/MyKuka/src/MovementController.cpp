/*
 * MovementController.cpp
 *
 *  Created on: 30 Mar 2019
 *      Author: Ben
 */

#include "MovementController.h"


void MovementController::setSerialPort(string path){
	serialPort.setPort(path);
}

void MovementController::setSerialBaud(int baud) {
	serialPort.setBaudrate(baud);
}

//void MovementController::setSerialTimeout(int timeout) {
////	Serial temp(serialPort.getPort(),serialPort.getBaudrate(),Timeout::simpleTimeout(timeout));
////	serialPort.setTimeout(Timeout::simpleTimeout(timeout));
//}

void MovementController::setMoveCircle(int c1, int c2, int c3, int rad, int step, int plane){
	cout<<"Set move - circle \n";
	sendFile << "Attempting circle set move"<<endl;
	sendFile << "Full Position 					Angles 					Result"<<endl;
	int size = 360/step;
	double a1[size], a2[size];
	switch(plane) {
	case 1:
		calcCircleCoords(c1,c3,rad,step,a1,a2);
		break;
	case 2:
		calcCircleCoords(c1,c2,rad,step,a1,a2);
		break;
	default:
		calcCircleCoords(c2,c3,rad,step,a1,a2);
		break;
	}

	vector<FullPosition> sendVec;
	for(int i=0; i<size; ++i) {
		MyPoint p;
		Rotation r(0,0,0);
		switch(plane) {
		case 1:
			p.set((double)a1[i],(double)c3,(double)a2[i]);
			break;
		case 2:
			p.set((double)a1[i],(double)a2[i],(double)c3);
			break;
		default:
			p.set((double)c3,(double)a1[i],(double)a2[i]);
			break;
		}
		FullPosition fp = FullPosition(p,r);
		kin.calcIK(fp);
		sendVec.push_back(fp);
	}
//	cout<<"Sending vector\n";
//	for(vector<FullPosition>::iterator it = sendVec.begin(); it != sendVec.end(); ++it) {
//		cout<<*it<<"\n";
//	}

	sendPositions(sendVec);
}

bool MovementController::sendPosition(FullPosition data) {
	if(!data.anglesSet()){
		if(!kin.calcIK(data)) return false;
	}
	if(data.isNull()) kin.calcFK(data);
	data.fixAlmostZero();
//	int angleJumpIdx;
//	while(angleJumpCheck(data.angles,currentPos.angles,angleJumpIdx)){
//		sendFile<<"***************Angle fix "<<angleJumpIdx<<"      ************************\n";
//		sendPositions(angleJumpPosFiller(data.angles,currentPos.angles,angleJumpIdx));
//		sendFile<<"***************Angle fix************************\n";
//	}

//	cout<<data;
//#ifdef CALIBRATION
//	data.angles[NO_ACTUATORS-1] = toRadians(1);
//#endif
	string sendStr = "<";
	for(int j=0; j<NO_ACTUATORS-1; ++j) {
		sendStr += to_string_with_precision(toDegrees(data.angles[j])) + ",";
	}
	sendStr+= to_string_with_precision<double>(toDegrees(data.angles[NO_ACTUATORS - 1])) + ">";
	size_t sendSize = sendStr.size();
	size_t bytesWrote = serialPort.write(sendStr);
	bool sendRes = false;
	if(sendSize == bytesWrote) {
		 sendRes = sendResult();
	}
//	cout<<"Sending \t"<<sendStr<<endl;
	sendFile << data.position <<"\t"<<data.orientation <<"			"<<data.angles<<" 		"<<sendRes<<endl;
	if(sendRes)	currentPos = data;
	return sendRes;
}

vector<int> MovementController::sendPositions(vector<FullPosition> data){
	vector<int> sendRes;
	for(int i=0; i<(int)data.size(); ++i) {
		bool res = sendPosition(data[i]);
		if(!res) sendRes.push_back(i);
	}
	return sendRes;
}

string MovementController::readSerial() {
	string temp, s;
	while(serialPort.read(temp,32)>0) s+=temp;
	return s;
}

bool MovementController::sendResult(){
	string buffer = serialPort.readline(65536,">");
//	cout<<buffer<<endl;
	if (buffer.find("<1>") != string::npos) {
	    return true;
	}
	return false;
}

void MovementController::calcCircleCoords(int c1, int c2, int radius, int step, double *_a1, double *_a2){
	for(int i=0; i<360; i+=step) {
		double angle = toRadians(i);
		_a1[i/step] = c1 + (radius * cos(angle));
		_a2[i/step] = c2 + (radius * sin(angle));
	}
}

bool MovementController::setTargetPosition(FullPosition targetPos){
//	cout<<targetPos<<endl;
	if(!targetPos.anglesSet()){
		if(!kin.calcIK(targetPos,false)) {
			cout<<"Invalid target\n";
			return false;
		}
	}

	bool resFlag = true;
	if(targetPos.position.isNull()) resFlag = sendPosition(targetPos);
	else {
		vector<FullPosition> targetVec = setTrajectoryAngles(targetPos,currentPos);
		vector<int> sendRes = sendPositions(targetVec);
		if(sendRes.size()>0) {
			for(auto it=sendRes.begin(); it != sendRes.end(); ++it) {
//				cout<<"Failure sending position "<< targetVec[(int)*it] <<endl;
			}
			if(sendRes.back()==(int)targetVec.size()-1) {
//				cout<<"Failure completing target move\n";
				resFlag = false;
			}
		}
	}
	cout<<"FInished target move with result \t"<<resFlag<<"\n";
	return resFlag;
//	return sendPosition(targetPos);
}

vector<FullPosition> MovementController::setTrajectoryAngles(FullPosition target, FullPosition current) {

	vector<FullPosition> targetVec;
	if((target.position == current.position && target.orientation == current.orientation)||(target.angles == current.angles)) {
		targetVec.push_back(target);
	} else {
		array<double,6> incVals;
		for(int i=0; i<NO_ACTUATORS-1;++i) {
				incVals[i] = (target.angles[i]-current.angles[i])/(double)TARGET_ITERS;
				almostZeroFix(incVals[i],incVals[i]);
		}
		sendFile<<"****************************\nCurrent\t"<<current<<endl;
		sendFile<<"Target\t"<<target<<"\n";
		for(int i=0;i<6;++i) {
			sendFile<<toDegrees(incVals[i]*TARGET_ITERS)<<"\t";
		}
		sendFile<<"\n****************************\n";

		for(int i=1; i<=TARGET_ITERS; ++i) {
			FullPosition temp;
			temp.null();
			for(int j=0;j<NO_ACTUATORS-1;++j) {
				temp.angles[j] = currentPos.angles[j] + i*incVals[j];
			}
			targetVec.push_back(temp);
		}
	}
	return targetVec;
}

vector<FullPosition> MovementController::setTrajectorySimple(FullPosition target, FullPosition current) {
	vector<FullPosition> targetVec;
	if(target.position == current.position && target.orientation == current.orientation) {
		targetVec.push_back(target);
	}
	else if((target.position[0] == HOME_X_POS && target.position[1] == HOME_Y_POS && target.position[2] == HOME_Z_POS) && current.angles.isNull()) {
		targetVec.push_back(target);
	} else {

		array<double,6> incVals;
		for(int i=0; i<NO_ACTUATORS-1;++i) {
			if(i < 3){
				incVals[i] = (double)(target.position[i] - current.position[i])/(double)TARGET_ITERS;
			} else {
				incVals[i] = (double)(target.orientation[i] - current.orientation[i])/(double)TARGET_ITERS;
			}
		}

//		kin.calcIK(current);
//		JointAngles curJA = current.angles;

		for(int i=1; i<=TARGET_ITERS; ++i) {
			FullPosition temp;
			temp.null();
			for(int j=0;j<NO_ACTUATORS-1;++j) {
				if(j < 3) temp.position[j] = (currentPos.position[j] + i*incVals[j]);
				else  temp.orientation[j] = (currentPos.orientation[j] + i*incVals[j]);
			}
//			kin.calcIK(temp);
//			int angleIdx;
//			if(!angleJumpCheck(temp.angles,curJA,angleIdx)) targetVec.push_back(temp);
////			while(angleJumpCheck(temp.angles,curJA,angleIdx)) {
////				sendFile<<"Fixing angle\t"<<angleIdx<<"\n";
////				vector<FullPosition> filler = angleJumpPosFiller(temp.angles,curJA,angleIdx);
////				targetVec.insert(targetVec.end(), filler.begin(), filler.end());
////				curJA = targetVec.back().angles;
////			}
			targetVec.push_back(temp);
//			curJA = targetVec.back().angles;
		}
	}
	return targetVec;
}

bool MovementController::initBlockGrab(FullPosition pos) {
	FullPosition blockLoc = FullPosition(pos);
//	blockLoc.position.x += 5;
	blockLoc.orientation.y = toRadians(90);
	blockLoc.position.z -= WRIST_LENGTH;
	blockLoc.angles.null();
	kin.calcIK(blockLoc, false);
	cout<<"Starting block grab with pos\t"<<blockLoc;
	return setTargetPosition(blockLoc);
}


void MovementController::generateCalibTargetFile(string fileName){
	ofstream file(fileName);
	srand(time(0));
	int count = 0;
	while(count < NO_POINTS) {
		double x,y,z;
		x = (rand() % (ARM_RADIUS+SHOULDER_OFFSET));
		y = (rand() % (ARM_RADIUS+SHOULDER_OFFSET))-((ARM_RADIUS+SHOULDER_OFFSET)*0.5);
		z = (rand() % (ARM_RADIUS+HIP_HEIGHT));
		if(withinSphereCheck(0,0,HIP_HEIGHT,x,y,z,ARM_RADIUS)) {
			FullPosition fp(x,y,z,0,0,0);
			if(kin.calcIK(fp, false)) {
				count++;
				file << x <<"\t"<< y << "\t"<< z << "\t"<< 0 << "\t"<< 0 << "\t"<< 0 <<"\n";
			}
		}
	}
}

vector<FullPosition> MovementController::angleJumpPosFiller(JointAngles target, JointAngles current, int pos) {
	vector<FullPosition> filler;
	int iters = (double)TARGET_ITERS/5;
	double inc = (target[pos] - current[pos])/iters;
	for(int i=1; i<=iters; ++i) {
		FullPosition temp;
		temp.angles = current;
		temp.angles[pos] = current[pos]+ i*inc;
		filler.push_back(temp);
	}
//	for(auto it=filler.begin(); it != filler.end(); ++it) {
//		cout<<*it<<"\n";
//	}
	return filler;
}

bool MovementController::angleJumpCheck(JointAngles target, JointAngles current, int& idx) {
	for(int i=0;i<NO_ACTUATORS; ++i){
		if(fabs(target.a[i]-current.a[i])>toRadians(TGT_ANGLE_THRESH)){
			idx = i;
			return true;
		}
	}
	return false;
}

double MovementController::calcBezierValue(double a, double supportA, double b, double supportB, double t) {
	return (1-t)*(1-t)*(1-t)*a + 3*t*(1-t)*(1-t)*supportA + 3*t*t*(1-t)*supportB + t*t*t*b;
}

vector<FullPosition> MovementController::getPositionsOnLine3d(FullPosition l1, FullPosition l2, int num) {
	vector<FullPosition> pos;
	double dist = distanceBetweenTwoPoints(l1.position.x,l1.position.y,l1.position.z,l2.position.x,l2.position.y,l2.position.z);
	double xVar = l1.position.x - l2.position.x;
	double yVar = l1.position.y - l2.position.y;
	double zVar = l1.position.z - l2.position.z;

	double rVar = l1.orientation.x - l2.orientation.x;
	double pVar = l1.orientation.y - l2.orientation.y;
	double ywVar = l1.orientation.z - l2.orientation.z;

	for(int i=0; i<=num; ++i) {
		double pInc = ((double)i/(double)num) * dist;
		double newX = l1.position.x+(xVar*(l1.position.x+pInc));
		double newY = l1.position.x+(yVar*(l1.position.x+pInc));
		double newZ = l1.position.x+(zVar*(l1.position.x+pInc));
		MyPoint p(newX,newY,newZ);

		double rInc = ((double)i/(double)num) * rVar;
		double ptchInc = ((double)i/(double)num) * pVar;
		double ywInc = ((double)i/(double)num) * ywVar;
		Rotation r(l1.orientation.x+rInc, l1.orientation.y+ptchInc, l1.orientation.z+ywInc);

		pos.push_back(FullPosition(p,r));
	}
	return pos;
}

bool MovementController::moveHome() {
	return setTargetPosition(FullPosition(MyPoint(HOME_X_POS,HOME_Y_POS,HOME_Z_POS),Rotation(0,0,0)));
//	return setTargetPosition(JointAngles());
}

void MovementController::enumeratePorts() {
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() ) {
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
	}
}


