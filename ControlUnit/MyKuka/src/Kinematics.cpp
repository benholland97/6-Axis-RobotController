/*
 * Kinematics.cpp
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#include "Kinematics.h"

void Kinematics::setup() {
    //                      alpha               r                      d
    DHParams[0] = DHParam(toRadians(-90),   SHOULDER_OFFSET,      HIP_HEIGHT);
    DHParams[1] = DHParam(0,                HUMERUS_LENGTH,       0);
    DHParams[2] = DHParam(toRadians(-90),   0,                    0);
    DHParams[3] = DHParam(toRadians(90),    0,                    FOREARM_LENGTH);
    DHParams[4] = DHParam(toRadians(-90),   0,                    0);
    DHParams[5] = DHParam(0,                0,                    WRIST_LENGTH);

	fillRotMatrix(toRadians(-90), toRadians(-90), toRadians(-90), fkCorRot);
	ikCorRot = fkCorRot;
	ikCorRot.inv();
}

JointAngles Kinematics::getCurrentJointAngles() {
	if(curAngles.isNull()) {
		curAngles.setDefaultPosition();
	}
	return curAngles;
}

void Kinematics::fillDHMatrix(double theta, DHParam dh, Matrix& m) {
	double ct = cos(theta);
	double st = sin(theta);

	double a = dh.getR(); 		// length of the actuator
	double sa = dh.sinalpha();	// precomputed for performance (alpha is constant)
	double ca = dh.cosalpha();	// precomputed for performance (alpha is constant)

	m = Matrix(4,4,
			{ ct, 	-st*ca,  st*sa,  	a*ct,
			  st, 	 ct*ca, -ct*sa,		a*st,
			  0,	 sa,		ca,		dh.getD(),
			  0,	 0,		     0,		1});
}

void Kinematics::fillRotMatrix(double x, double y, double z, Matrix& m) {
	double sinX = sin(x);
	double cosX = cos(x);
	double sinY = sin(y);
	double cosY = cos(y);
	double sinZ = sin(z);
	double cosZ = cos(z);

	m = Matrix(4,4,
			{ 	cosZ*cosY, 	-sinZ*cosX+cosZ*sinY*sinX,  	sinZ*sinX+cosZ*sinY*cosX, 	0,
				sinZ*cosY, 	 cosZ*cosX + sinZ*sinY*sinX, 	cosZ*sinX+sinZ*sinY*cosX, 	0,
				-sinY,	 	cosY*sinX,						cosY*cosX,					0,
				0,			0,								0,							1});
}

void Kinematics::fillTransMatrix(MyPoint p, Rotation r, Matrix& m) {
	double sinx = sin(r[0]);
	double cosx = cos(r[0]);
	double siny = sin(r[1]);
	double cosy = cos(r[1]);
	double sinz = sin(r[2]);
	double cosz = cos(r[2]);

	// Set transformation matrix T06
	// left upper 3x3 part is rotation matrix out of three euler angles in zy'x'' model
	m = Matrix(4,4,
		{ 	cosz*cosy,	cosz*siny*sinx-sinz*cosx,	cosz*siny*cosx+sinz*sinx,	p[0],
			sinz*cosy,	sinz*siny*sinx+cosz*cosx,	sinz*siny*cosx-cosz*sinx,	p[1],
			-siny,		cosy*sinx,					cosy*cosx,					p[2],
			0,			0,							0,							1 });
}

void Kinematics::calcFK(FullPosition& fp) {
	setup();
    Matrix T01,T12,T23,T34,T45,T56;
    fillDHMatrix(fp.angles[0],DHParams[0],T01);
    fillDHMatrix(-toRadians(90)+fp.angles[1],DHParams[1],T12);
    fillDHMatrix(fp.angles[2],DHParams[2],T23);
    fillDHMatrix(fp.angles[3],DHParams[3],T34);
    fillDHMatrix(fp.angles[4],DHParams[4],T45);
    fillDHMatrix(fp.angles[5],DHParams[5],T56);

//    cout<<T01<<"\n"<<T12<<"\n"<<T23<<"\n"<<T34<<"\n"<<T45<<"\n"<<T56<<"\n";

    Matrix T06 = T01*T12*T23*T34*T45*T56;

    T06 *= fkCorRot;

	double beta = atan2(-T06[2][0], sqrt(T06[0][0]*T06[0][0] + T06[1][0]*T06[1][0]));
	double gamma = 0;
	double alpha = 0;
	if (almostEqual(beta, PI_2, FLOAT_PRECISION)) {
		alpha = 0;
		gamma = atan2(T06[0][1], T06[1][1]);
	} else {
			if (almostEqual(beta, -PI_2,FLOAT_PRECISION)) {
				alpha = 0;
				gamma = -atan2(T06[0][1], T06[1][1]);
			} else {
				alpha = atan2(T06[1][0],T06[0][0]);
				gamma = atan2(T06[2][1], T06[2][2]);
			}
	}

	almostZeroFix(T06, FLOAT_PRECISION);

	fp.position = T06.column(3);

	fp.orientation[0] = gamma;
	fp.orientation[1] = beta;
	fp.orientation[2] = alpha;

//	curAngles = fp.angles;
}

bool Kinematics::calcIK(FullPosition& fp, bool print, bool setAngles) {
	setup();
	IKCandidates solutions;
	calcIKSolutions(fp, solutions);

	if(print) {
		cout<<"*******************\nKinematics"<<endl;
		for(int i=0; i<solutions.size(); ++i) {
			cout<<solutions[i]<<endl;
		}
		cout<<"*******************"<<endl;
	}


	if(solutionValidityCheck(solutions)) {

		if(solutions.size() == 1) {
			fp.angles = solutions[0].ja;
		} else {
			fp.angles = optimalSolutionCheck(solutions).ja;
		}
		almostZeroFix(fp.angles.a, NO_ACTUATORS, FLOAT_PRECISION);
//		cout<<"Kinematics \n"<<fp<<endl;
//		cout<<fp;

	} else {
		return false;
	}
	if(setAngles) {
		curAngles = fp.angles;
	}
	return true;
}

void Kinematics::calcIKSolutions(FullPosition& fp, IKCandidates& solutions) {
	//Calc T06 from TCP
	Matrix T06;
	fillTransMatrix(fp.position,fp.orientation,T06);

	T06*= ikCorRot;

	Vector TCP_to_WCP = { 0,0,-WRIST_LENGTH,1 };
	Vector wcp = T06 * TCP_to_WCP;

	//angle 0
	double a0_1 = atan2(wcp[1],wcp[0]);
	double a0_2 = atan2(-wcp[1],-wcp[0]);

	//	// singularity check: if we are right above the origin, keep angle 0
	//	if ((fabs(wcp[1]) < double_PRECISION) &&  (fabs(wcp[0]) < double_PRECISION)) {
	//		angle0_solution1 = curAngles[0];
	//		angle0_solution2 = PI_2 - curAngles[0];
	//	}

	bool tcpXPositive = fp.position[0] >= 0;
	double a0_fwd = 0;
	double a0_bck = 0;

	if (tcpXPositive) {
		a0_fwd =  a0_1;
		a0_bck = a0_2;
	} else {
		a0_fwd =  a0_2;
		a0_bck = a0_1;
	}

	//angles 1 + 2
    //using triangle (joint1(A=base + hip height), joint2(c) and joint3(WCP)
    // a = humerus length, b = forearm length, c = distance wcp[x] to wcp[y]

    double zJ1ToWCP = wcp[2] - HIP_HEIGHT;
    //from aerial view
    double lenBaseToWCP = hypotenuseLength(wcp[0],wcp[1]) - SHOULDER_OFFSET;

    double c = hypotenuseLength(zJ1ToWCP,lenBaseToWCP);
    double a = FOREARM_LENGTH;
    double b = HUMERUS_LENGTH;
    double alpha = acos((a*a - b*b - c*c)/(-2*b*c));
    double gamma = acos((c*c - a*a - b*b)/(-2*a*b));

    //flags to store flip/non flip orientation of triangle
    int flipFlag_fwd = tcpXPositive?  1 : -1;
    int flipFlag_bck = tcpXPositive? -1 :  1;

    double delta_fwd = atan2(zJ1ToWCP,flipFlag_fwd*lenBaseToWCP);
    double delta_bck = atan2(zJ1ToWCP,flipFlag_bck*lenBaseToWCP);

	double a1_fwd_1 = PI_2 - ( delta_fwd + alpha);
	double a1_fwd_2 = PI_2 - ( delta_fwd - alpha);
	double a1_bck_1 = PI_2 - ( delta_bck + alpha);
	double a1_bck_2 = PI_2 - ( delta_bck - alpha);

	double a2_1 = PI_2 - gamma;
	double a2_2 = gamma - (PI*3.0/2.0);

	//extract spherical wrist rotation angles
	calcSphericalWristAngles(IKCandidates::BaseDirection::FORWARD, IKCandidates::ArmConfig::NO_FLIP,
			a0_fwd, a1_fwd_1, a2_1, T06, solutions);

	calcSphericalWristAngles(IKCandidates::BaseDirection::FORWARD, IKCandidates::ArmConfig::FLIP,
			a0_fwd, a1_fwd_2, a2_2, T06, solutions);

	calcSphericalWristAngles(IKCandidates::BaseDirection::BACKWARD, IKCandidates::ArmConfig::NO_FLIP,
			a0_bck, a1_bck_1, a2_1, T06, solutions);

	calcSphericalWristAngles(IKCandidates::BaseDirection::BACKWARD, IKCandidates::ArmConfig::FLIP,
			a0_bck, a1_bck_2, a2_2, T06, solutions);

}

void Kinematics::calcSphericalWristAngles(IKCandidates::BaseDirection dir, IKCandidates::ArmConfig conf,
			double a0, double a1, double a2, Matrix T06, IKCandidates& solutions) {
//	IKCandidates::Candidate sUp;
//	IKCandidates::Candidate sDwn;

	IKSol sUp, sDwn;
	sUp.a = dir;
	sUp.b = conf;
	sUp.c = IKCandidates::HandOrientation::UP;

	sUp.ja[0] = a0;
	sUp.ja[1] = a1;
	sUp.ja[2] = a2;

	sDwn = sUp;
	sDwn.c = IKCandidates::HandOrientation::DOWN;

	//angles 3 4 5
	Matrix T01, T12, T23, T03;
	fillDHMatrix(a0,DHParams[0],T01);
	fillDHMatrix(a1-toRadians(90),DHParams[1],T12);
	fillDHMatrix(a2,DHParams[2],T23);

	Matrix R01 = T01[mslice(0,0,3,3)];
	Matrix R12 = T12[mslice(0,0,3,3)];
	Matrix R23 = T23[mslice(0,0,3,3)];

	//calc split up on walter
	Matrix R03 = R01*R12*R23;
	// compute inverse of R03 by transposing manually
	Matrix R03_inv(R03);
	mswap(R03_inv[0][1], R03_inv[1][0]);
	mswap(R03_inv[0][2], R03_inv[2][0]);
	mswap(R03_inv[1][2], R03_inv[2][1]);

	Matrix R06 = T06[mslice(0,0,3,3)];
	Matrix R36 = R03_inv*R06;

	double R36_22 = R36[2][2];
	double R36_01 = R36[0][1];

	// sometimes, R36_22 is slightly greater than 1 due to floating point arithmetics
	// since we call acos afterwards, we need to compensate that, otherwise we got an invalid result.
	if ((fabs(R36_22) > 1.0) && (fabs(R36_22) < (1.0+FLOAT_PRECISION))) {
		R36_22 = (R36_22>0)?1.0:-1.0;
	}

	if ((fabs(R36_01) > 1.0) && (fabs(R36_01) < (1.0+FLOAT_PRECISION))) {
		R36_01 = (R36_01>0)?1.0:-1.0;
	}

	sUp.ja[4] = acos(R36_22);
	sDwn.ja[4] = -acos(R36_22);

	double sinA4_1 = sin(sUp.ja[4]);
	double sinA4_2 = sin(sDwn.ja[4]);

	// if wrist is 0�, there is an infinite number of solutions (singularity)
	// this requires a special treatment that keeps angles close to curAngles position
	if ((sinA4_1*sinA4_1) < FLOAT_PRECISION) {

//		cout<<"Gimbal singularity fix";

		// keep angle 4 (elbow) stable and move the others only
		sUp.ja[3]   = curAngles[3];
		sDwn.ja[3] = curAngles[3];

		double asinR36_01 = asin(-R36_01);
		sUp.ja[5]  = asinR36_01  - sUp.ja[3];
		sDwn.ja[5]= asinR36_01  - sDwn.ja[3];

        // normalise angles by bring it in an interval -PI..PI
        while ((abs( sUp.ja[5] - curAngles[5]) > abs( sUp.ja[5] + PI - curAngles[5])) &&
        		(sUp.ja[5] + PI <= MG90S_MAX_ANGLE)) {
        	sUp.ja[5]   += PI;
        	sDwn.ja[5] += PI;
        }
       	while ((abs( sUp.ja[5] - curAngles[5]) >
             abs( sUp.ja[5] - PI - curAngles[5])) &&
        	(sUp.ja[5] - PI >= MG90S_MIN_ANGLE)) {
       		sUp.ja[5]   -= PI;
            sDwn.ja[5] -= PI;
        }
	} 	else {
		// from here, sin_angle_4_x is not close to 0, so the following works
		sUp.ja[5]   = atan2( - R36[2][1]/sinA4_1, R36[2][0]/sinA4_1);
		sDwn.ja[5] = atan2( - R36[2][1]/sinA4_2, R36[2][0]/sinA4_2);

		sUp.ja[3]   = -atan2( R36[1][2]/sinA4_1,- R36[0][2]/sinA4_1);
		sDwn.ja[3] = -atan2( R36[1][2]/sinA4_2,- R36[0][2]/sinA4_2);

	}

	solutions.add(sUp);
	solutions.add(sDwn);
}

bool Kinematics::solutionValidityCheck(IKCandidates& solutions) {
//	cout<<"Number of beginning solutions: "<<solutions.size()<<"\n";
	for(int i=0; i<solutions.size(); ++i) {
		if(!solutionValidityCheck(solutions[i])) {
//			cout<<"removing solution "<<i<<"\n";
			solutions.remove(i);
			i--;
		}
	}
//	cout<<"Number of remaining solutions: "<<solutions.size()<<"\n";
	return solutions.size()>0?true:false;
}

bool Kinematics::solutionValidityCheck(IKSol sol) {
//	cout<<"checking single solution : result "<<sol.ja.limitsCheck()<<"\n";
	return sol.ja.limitsCheck();
}

IKSol Kinematics::optimalSolutionCheck(IKCandidates& solutions) {
	double minDiff(1<<20);
	int idx(0);
	for(int i=0; i<solutions.size(); ++i) {
		double newDiff = solutions[i].ja.distanceBetween(curAngles);
		if(newDiff < minDiff) idx = i;
	}
	return solutions[idx];
}

void Kinematics::testSolution(FullPosition pos, IKCandidates solutions) {
	for(int i=0; i<solutions.size();++i) {
		testSolution(pos,solutions[i]);
	}
}

void Kinematics::testSolution(FullPosition pos, IKSol sol) {
	setup();
	FullPosition tPos(sol.ja);
	calcFK(tPos);

	cout<<"Forward Calculated Pos: " <<tPos<<"\n";
	double diff = pos.compare(tPos);
	almostZeroFix(diff,FLOAT_PRECISION);

	cout<<"Solution values: "<<sol;
	cout<<"Forward Kin angles check difference :\t: "<<diff<<"\n\n";


}

ostream& operator<<(ostream& os, const IKSol& iks) {
	IKSol t = iks;
	t.ja.fixAlmostZero();
	os<<"Direction : "<<t.a<<"\tConfiguration : "<<t.b<<"\tOrientation : "<<t.c<<"\n";
	os<<"Angles : "<<t.ja <<"\n";
	return os;
}

ostream& operator<<(ostream& os, const IKCandidates& ikc) {
	IKCandidates t = ikc;
	os<<"Solutions: \n";
	for(int i=0; i<t.size();++i) {
		t[i].ja.fixAlmostZero();
		os<<"Direction : "<<t[i].a<<"\tConfiguration : "<<t[i].b<<"\tOrientation : "<<t[i].c<<"\n";
		os<<"Angles : "<<t[i].ja <<"\n";
	}
	return os;
}

