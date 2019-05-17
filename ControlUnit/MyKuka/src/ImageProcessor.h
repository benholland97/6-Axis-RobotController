/*
 * ImageProcessor.h
 *
 *  Created on: 7 Apr 2019
 *      Author: Ben
 */

#ifndef SRC_IMAGEPROCESSOR_H_
#define SRC_IMAGEPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "Utils.h"
#include "Config.h"
#include "MovementController.h"
#include <thread>
#include <future>
#include <chrono>

using namespace std;
using namespace cv;

struct RectWithType {
	RectWithType(){};
	RectWithType(RotatedRect _r, int _t){
		rect = _r;
		type = _t;
	}
	RectWithType(const RectWithType& _r) {
		rect = _r.rect;
		type = _r.type;
		targetArea = _r.targetArea;
	}

	RectWithType& operator=(RectWithType rhs)
	{
		rect = rhs.rect;
		type = rhs.type;
		targetArea = rhs.targetArea;
		return *this;
	}

    friend ostream& operator<<(ostream& os, const RectWithType& p);

	RotatedRect rect;
	int type;
	double targetArea = 0;
};

struct TrackbarJointAngleData {
	double angles[NO_ACTUATORS];// = {MG90S_MAX_ANGLE,MG90S_MAX_ANGLE,MG90S_MAX_ANGLE,MG90S_MAX_ANGLE,MG90S_MAX_ANGLE,MG90S_MAX_ANGLE};
	bool isReady = false;
	void setAngle(int num, int val) {
		if(num >= 0 && num < NO_ACTUATORS) {
			int offset = MG90S_MAX_ANGLE;
			if(num == NO_ACTUATORS-1) {
				offset = 0;
			}
			if(num < NO_MG996R) offset = MG996R_MAX_ANGLE;
			angles[num] = val - offset;
		}
	}
};

enum ActuatorNum {
	A0,A1,A2,A3,A4,A5,A6
};


class ImageProcessor {
public:

	ImageProcessor(int camIdx, string path= "res/calib.xml");

	ImageProcessor(string path= "res/calib.xml");

	~ImageProcessor(){
		cap->release();
	}

	bool ready(){
		return cap->isOpened() && rotMatSet() && camMatSet() &&tVecSet();
	}

	int detectByColour(vector<Point2f>& res, int hueCentre= HUE_RED_CNTR, int hueRange= HUE_RED_RNGE);

	bool userAction(MovementController& mc);

	bool pixelToWorldCoordinates(Point2f pixel, Point3f &world);

	void worldToPixelCoordinates(vector<Point3f> objPoints, vector<Point2f> &imgPoints, Mat _rvec, Mat _tvec, Mat _camMatrix, Mat _distCoeffs);

	bool calcPositionByAruco(int cornerRef=1);

	void setCamMat(String path = "res/calib.xml"){
		readCameraParameters(path,camMatrix, distCoeffs);
	}

	void setCamMat(Mat _cm) {
		camMatrix(_cm);
	}

	bool rotMatSet() {
		return !rotMatrix.empty();
	}

	bool camMatSet() {
		return !(camMatrix.empty() && distCoeffs.empty());
	}

	bool tVecSet() {
		return !transMatrix.empty();
	}

	Mat getRotMat() {
		return rotMatrix;
	}

	void mousePressCallback(int event, int x, int y);

	static void mousePressCallback(int event, int x, int y, int, void* userdata);

	static void trackBarJointAngles(int val, void* userdata);

	static void trackbarCallback(int, void* userdata);

	static bool rectDetect(vector<RotatedRect>& rects, Mat img, int cannyThresh, Mat& imgOut);

	static bool sortRectVecAscArea(RectWithType lhs, RectWithType rhs);

	static bool sortRectNearestArea(RectWithType lhs, RectWithType rhs);

	static bool sortRectsByType(RectWithType lhs, RectWithType rhs);


	bool isReadyForNewImage(){
		return readyForNewImgFlag;
	}

	void setReadyFlag(bool flag) {
		readyForNewImgFlag = flag;
	}


private:

	bool minSizeRectCheck(RotatedRect temp, double pWidth, double pDepth);

	bool squareCheck(RectWithType rect, int tolerance = BLOCK_TOLERANCE*0.25);

	vector<Point2f> calcIdealBlockFacesImg(double& pWidth, double& pHeight, double& pDepth, double blockWidth= BLOCK_WIDTH, double blockHeight=BLOCK_LENGTH, double blockDepth=BLOCK_DEPTH);

	Mat drawIdealBlock(Mat imgSrc, vector<Point2f> points);

	void targetSetDelay(unsigned int delayms);

	Mat drawTarget(Mat imgSrc, RotatedRect rect);

	FullPosition rectToRealPos(RectWithType rect);

	bool optimalRectSearch(vector<RectWithType> rects, RectWithType& opt,double blockWidth, double blockHeight, double blockDepth);

	bool filterBlockDetection(vector<RotatedRect> inRects, RectWithType& idealRect, Mat imgSrc, double pWidth, double pHeight, double pDepth, int tolerance=BLOCK_TOLERANCE);

	bool withinArucoMarkerCheck(Point3f point, Point3f centre = Point3f(ARUCO_YOFFSET,ARUCO_XOFFSET,0), double markerLength = MARKER_LENGTH,int tolerance=BLOCK_TOLERANCE);

	bool withinArucoPixelCheck(RotatedRect point, vector<Point2f> corners);

	//	void removeRectsBySize(vector<RotatedRect>inContours,vector<RotatedRect>&outContours, double length, double width, int tolerance = 5);

	bool multiCamConnectAttempt(int upperBound=5);

	bool connectCam(int idx);

	bool readCameraParameters(string filename, Mat &camMat, Mat &distCoeffs);

	bool readCalibTargets(vector<FullPosition>& targets, string fileName="res/calibTargets.txt");

	vector<Point3f> getCornersInCameraWorld(double _side, Vec3d _rvec, Vec3d _tvec);

	void calcExtrinsicParams(vector<Point2f> imgPoints, vector<Point3f> objPoints, Mat& _rvec, Mat& _tvec, Mat& _rotMatrix);

	void calcExtrinsicParamsAruco(vector<Point2f> corners, Mat& _rvec, Mat& _tvec, Mat& _rotMat, Point3f centre = Point3f(0,0,0), double markerLength = MARKER_LENGTH);

	Mat rvec, tvec;
	Mat camMatrix, rotMatrix, distCoeffs, transMatrix;
	vector<Point2f> calibArucoCorners;
	shared_ptr<VideoCapture> cap;
	double sfWP;
	ofstream rectfile;
	atomic<bool> readyForNewImgFlag, homeMoveFlag;
	FullPosition targetPos;
	bool targetSet, autoMode;
#ifdef ARM_CALIBRATION
	vector<FullPosition> calibTargets;
	int calibIdx;
#endif


};

struct ImageProcessorCallback {

	ImageProcessorCallback(){};
	ImageProcessorCallback(Mat _img){
		img = _img;
	}
	vector<RotatedRect> rects;
	Mat img, imgOut;
	int thresh = CANNY_THRESH;
};

#endif /* SRC_IMAGEPROCESSOR_H_ */
