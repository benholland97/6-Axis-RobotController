/*
 * Kinematics.h
 *
 *  Created on: 11 Mar 2019
 *      Author: Ben
 */

#ifndef SRC_KINEMATICS_H_
#define SRC_KINEMATICS_H_

#include "DHParams.h"
#include "Positions.h"
#include "Utils.h"
#include "Config.h"


class IKCandidates{
public:
	enum BaseDirection {FORWARD,BACKWARD,D_NOT_SET};
	enum ArmConfig {FLIP,NO_FLIP,C_NOT_SET};
	enum HandOrientation {UP,DOWN,O_NOT_SET};

	struct Candidate {
		Candidate():a(D_NOT_SET),b(C_NOT_SET),c(O_NOT_SET) {};
		Candidate(JointAngles _ja, BaseDirection _a, ArmConfig _b):ja(_ja),a(_a),b(_b),c(O_NOT_SET){};

		void operator= (const Candidate& x) {
			ja = x.ja;
			a = x.a;
			b = x.b;
			c = x.c;
		}

		JointAngles ja;
		BaseDirection a;
		ArmConfig b;
		HandOrientation c;

		friend ostream& operator<<(ostream& os, const Candidate& iks);

	};

	IKCandidates() {}

	IKCandidates(const IKCandidates& a) {
		c = a.c;
	}

	int size() {
		return (int)c.size();
	}

    Candidate& operator[](int idx) {
    	return c[idx];
//		if ((idx >= 0) || ( idx < (int)c.size()))
//			return c[idx];
	}

    const Candidate&  operator[](int idx) const{
    	return c[idx];
//		if ((idx >= 0) || ( idx < (int)c.size()))
//			return c[idx];
	}

	void add(JointAngles _ja, BaseDirection _a, ArmConfig _b) {
		c.push_back(Candidate(_ja,_a,_b));
	}

	void add(Candidate _c) {
		c.push_back(_c);
	}

	void remove(int idx) {
		c.erase(c.begin() + idx);
	}

    friend ostream& operator<<(ostream& os, const IKCandidates& ikc);


private:
	vector<Candidate> c;
};


typedef IKCandidates::Candidate  IKSol;


class Kinematics {
public:
	Kinematics() {
		setup();
	}

	Kinematics(const Kinematics& _k){
		setup();
		curAngles = _k.curAngles;
	}

	void setup();

	void calcFK(FullPosition& fp);

	bool calcIK(FullPosition& fp, bool print = false, bool setAngles = true);

	JointAngles getCurrentJointAngles();

private:
	DHParam DHParams[NO_ACTUATORS];
	Matrix fkCorRot, ikCorRot;
	JointAngles curAngles;

	void fillDHMatrix(double theta, DHParam dh, Matrix& m);
	void fillRotMatrix(double x, double y, double z, Matrix& m);
	void fillTransMatrix(MyPoint p, Rotation r, Matrix& m);

	void calcIKSolutions(FullPosition& fp, IKCandidates& solutions);

	void calcSphericalWristAngles(IKCandidates::BaseDirection dir, IKCandidates::ArmConfig conf,
			double a0, double a1, double a2, Matrix m,  IKCandidates& solutions);

	void testSolution(FullPosition pos, IKCandidates solutions);

	void testSolution(FullPosition fp, IKSol sol);

	bool solutionValidityCheck(IKCandidates& solutions);

	bool solutionValidityCheck(IKSol sol);

	IKSol optimalSolutionCheck(IKCandidates& solutions);

};

#endif /* SRC_KINEMATICS_H_ */
