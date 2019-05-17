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

using namespace std;
using namespace cv;



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

	bool userAction();

	bool pixelToWorldCoordinates(Point2f pixel, Point3f &world, float z=0);

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

//	static void trackbarTolerance(int, void* userdata);

	static void trackbarCallback(int, void* userdata);

	static bool rectDetect(vector<RotatedRect>& rects, Mat img, int cannyThresh, Mat& imgOut);



private:
	void filterBlockDetection(vector<RotatedRect> inRects, vector<RotatedRect>& outRects, Mat imgSrc, Mat& imgOut, double blockWidth= BLOCK_WIDTH, double blockHeight=BLOCK_LENGTH, double blockDepth=BLOCK_DEPTH, int tolerance=BLOCK_TOLERANCE);

	bool withinArucoMarkerCheck(Point3f point, Point3f centre = Point3f(ARUCO_YOFFSET,ARUCO_XOFFSET,0), double markerLength = MARKER_LENGTH,int tolerance=BLOCK_TOLERANCE);

	bool withinArucoPixelCheck(RotatedRect point, vector<Point2f> corners);

	//	void removeRectsBySize(vector<RotatedRect>inContours,vector<RotatedRect>&outContours, double length, double width, int tolerance = 5);

	bool multiCamConnectAttempt(int upperBound=5);

	bool connectCam(int idx);

	bool readCameraParameters(string filename, Mat &camMat, Mat &distCoeffs);

	vector<Point3f> getCornersInCameraWorld(double _side, Vec3d _rvec, Vec3d _tvec);

	void calcExtrinsicParams(vector<Point2f> imgPoints, vector<Point3f> objPoints, Mat& _rvec, Mat& _tvec, Mat& _rotMatrix);

	void calcExtrinsicParamsAruco(vector<Point2f> corners, Mat& _rvec, Mat& _tvec, Mat& _rotMat, Point3f centre = Point3f(0,0,0), double markerLength = MARKER_LENGTH);


	Mat rvec, tvec;
	Mat camMatrix, rotMatrix, distCoeffs, transMatrix;
	vector<Point2f> calibArucoCorners;
	shared_ptr<VideoCapture> cap;
	double sfWP;
	ofstream rectfile;

};

struct ImageProcessorCallback {

	ImageProcessorCallback(){};
	ImageProcessorCallback(Mat _img){
		img = _img;
	}
	vector<RotatedRect> rects;
	Mat img, imgOut;
	int thresh = 100;
};

#endif /* SRC_IMAGEPROCESSOR_H_ */
