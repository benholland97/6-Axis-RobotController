/*
 * ImageProcessor.cpp
 *
 *  Created on: 7 Apr 2019
 *      Author: Ben
 */

#include "ImageProcessor.h"


TrackbarJointAngleData tbJointAngles;


ImageProcessor::ImageProcessor(string path) {
	if(multiCamConnectAttempt()) {
		setCamMat(path);
//		readyForNewImgFlag = true;
	}
}

ImageProcessor::ImageProcessor(int camIdx, string path) {
	if(connectCam(camIdx)) {
		setCamMat(path);
//		readyForNewImgFlag = true;
	}
}

bool ImageProcessor::multiCamConnectAttempt(int upperBound){
	for(int i=0; i<upperBound; ++i) {
		if(connectCam(i)) return true;
	}
	cout<<"Unable to connect camera\n";
	return false;
}

bool ImageProcessor::connectCam(int idx) {
	cap = make_shared<VideoCapture>(idx);
	// Check if camera opened successfully
	if(!cap->isOpened()){
		cout << "Error opening video stream or file index" << idx <<"\n";
		cap.reset();
		return false;
	}
	return true;
}

int ImageProcessor::detectByColour(vector<Point2f>& res, int hueCentre, int hueRange){
	cout<<"Starting colour detection for hue values between "<<(hueCentre-hueRange)%180 <<" and "<< (hueCentre+hueRange)%180<<"\n";
	char charCheckForEscKey = 0;
    const String srcWindow = "Source";
    const String maskWindow = "Mask";

    namedWindow(srcWindow);
    namedWindow(maskWindow);

    Mat frame, frameHSV, frameThreshold;

	while (charCheckForEscKey != 27 && cap->isOpened()) {
		bool frameReadFlag = cap->read(frame);
		if(!frameReadFlag || frame.empty()) {
			cout<<"Error reading frame\n";
			return -1;
		}
//		medianBlur(frame, frame, 3);
		cvtColor(frame, frameHSV, COLOR_BGR2GRAY);

		if(hueCentre >= 170 && hueCentre <= 180) {
			Mat thresh1, thresh2;
//			inRange(frameHSV, Scalar((hueCentre-hueRange)%180,HSV_MIN_VAL,HSV_MIN_VAL),Scalar(180,HSV_MAX_VAL,HSV_MAX_VAL),thresh1);
//			inRange(frameHSV, Scalar(0,HSV_MIN_VAL,HSV_MIN_VAL),Scalar((hueCentre+hueRange)%180,HSV_MAX_VAL,HSV_MAX_VAL),thresh2);
			inRange(frameHSV, Scalar(0,70,100),Scalar(10,255,255),thresh1);
			inRange(frameHSV, Scalar(170,70,100),Scalar(179,255,255),thresh2);
			frameThreshold = thresh1 | thresh2;
//			addWeighted(thresh1, 1.0, thresh2, 1.0, 0.0, frameThreshold);


		} else {
			inRange(frameHSV, Scalar((hueCentre-hueRange)%180,HSV_MIN_VAL,HSV_MIN_VAL),Scalar((hueCentre+hueRange)%180,HSV_MAX_VAL,HSV_MAX_VAL),frameThreshold);
		}

		GaussianBlur(frameThreshold, frameThreshold, cv::Size(3, 3), 0);			//Blur Effect
		dilate(frameThreshold, frameThreshold, 0);								// Dilate Filter Effect
		erode(frameThreshold, frameThreshold, 0);

        imshow("Source", frame);
        imshow("Mask", frameThreshold);

	    charCheckForEscKey = waitKey(1);


    }
	cap->release();
	return 1;
}

bool ImageProcessor::calcPositionByAruco(int cornerRef){
	cout<<"Calculation camera position by aruco marker\n";

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = cornerRef;

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(ARUCO_DICT));

    bool readOk = camMatSet();
    if(!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }

    int waitTime = 10, totalIterations = 0;
    double totalTime = 0;
    namedWindow("Aruco");

    while(cap->grab()) {
    	Mat distImage, image, imageCopy;
//    	cap->retrieve(image);
    	cap->retrieve(distImage);
    	undistort(distImage,image,camMatrix,distCoeffs);
    	double tick = (double)getTickCount();

    	vector<int> ids;
    	vector<vector<Point2f>> corners;
    	vector<Vec3d> rVecs, tVecs;

    	rvec.create(1,3,cv::DataType<double>::type);
    	tvec.create(1,3,cv::DataType<double>::type);
    	rotMatrix.create(3,3,cv::DataType<double>::type);

    	aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);
//    	if(ids.size()>0 && find(ids.begin(), ids.end(), 1) != ids.end()) {
    	if(ids.size()>0 ) {
    		calcExtrinsicParamsAruco(corners[0],rvec, tvec, rotMatrix, Point3f(ARUCO_YOFFSET,ARUCO_XOFFSET+SHOULDER_OFFSET,0));

//            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rVecs, tVecs);
//            vector<Point3f> cornersWorld = getCornersInCameraWorld(markerLength, rVecs[0], tVecs[0]);
//            for(int i=0; i<(int)cornersWorld.size(); ++i) {
//            	cout<<"Corner "<<i<<" "<<cornersWorld[i]<<endl;
//            }
//            cv::aruco::drawDetectedMarkers(image, corners, ids);
            destroyWindow("Aruco");
            return true;
    	}

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        imshow("Aruco",image);

        char key = (char)waitKey(waitTime);
        if(key == 27) break;

    }
    return false;
}

void ImageProcessor::calcExtrinsicParamsAruco(vector<Point2f> corners, Mat& _rvec, Mat& _tvec, Mat& _rotMatrix, Point3f centre, double markerLength){
	vector<Point3f> objPoints;
	float halfLength(markerLength*0.5);

	objPoints.push_back(Point3f(centre.x-halfLength,centre.y-halfLength,centre.z));
	objPoints.push_back(Point3f(centre.x+halfLength,centre.y-halfLength,centre.z));
	objPoints.push_back(Point3f(centre.x+halfLength,centre.y+halfLength,centre.z));
	objPoints.push_back(Point3f(centre.x-halfLength,centre.y+halfLength,centre.z));

	sfWP = norm(corners[0]-corners[1])/markerLength;
	calibArucoCorners = corners;

	cout<<"Scale factor world to pixel is "<<sfWP<<endl;

	cout<<"World cords of markers \n"<<endl;
	for(int i=0; i<(int)objPoints.size(); ++i){
		cout<<objPoints[i]<<endl;
	}
	cout<<"\n";
	for(int i=0; i<(int)objPoints.size(); ++i){
		cout<<corners[i]<<endl;
	}


	calcExtrinsicParams(corners, objPoints, _rvec, _tvec, _rotMatrix);
}

void ImageProcessor::calcExtrinsicParams(vector<Point2f> imgPoints, vector<Point3f> objPoints, Mat& _rvec, Mat& _tvec, Mat &_rotMatrix) {
	solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, _rvec, _tvec);
	Rodrigues(_rvec,_rotMatrix);
}

vector<Point3f> ImageProcessor::getCornersInCameraWorld(double side, Vec3d rvec, Vec3d tvec){
     double half_side = side/2;

     // compute rot_mat
     Mat rot_mat;
     Rodrigues(rvec, rot_mat);

     // transpose of rot_mat for easy columns extraction
     Mat rot_mat_t = rot_mat.t();

     // the two E-O and F-O vectors
     double * tmp = rot_mat_t.ptr<double>(0);
     Point3f camWorldE(tmp[0]*half_side,
                       tmp[1]*half_side,
                       tmp[2]*half_side);

     tmp = rot_mat_t.ptr<double>(1);
     Point3f camWorldF(tmp[0]*half_side,
                       tmp[1]*half_side,
                       tmp[2]*half_side);

     // convert tvec to point
     Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

     // return vector:
     vector<Point3f> ret(4,tvec_3f);

     ret[0] +=  camWorldE + camWorldF;
     ret[1] += -camWorldE + camWorldF;
     ret[2] += -camWorldE - camWorldF;
     ret[3] +=  camWorldE - camWorldF;

     return ret;
}

bool ImageProcessor::pixelToWorldCoordinates(Point2f pixel, Point3f &world){
//	if(!ready()) return false;

	Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<double>::type); //u,v,1
	uvPoint.at<double>(0,0) = (double) pixel.x;
	uvPoint.at<double>(1,0) = (double) pixel.y;
	uvPoint.at<double>(2,0) = 1.0;

	Mat tempMat, tempMat2;
	double s, zConst = 0;

	tempMat = rotMatrix.inv() * camMatrix.inv() * uvPoint;
	tempMat2 = rotMatrix.inv() * tvec;
	s = zConst + tempMat2.at<double>(2,0);
	s /= tempMat.at<double>(2,0);
	Mat wcPoint = rotMatrix.inv() * (s * camMatrix.inv() * uvPoint - tvec);

	Point3f realPoint(wcPoint.at<double>(1, 0), wcPoint.at<double>(0, 0), wcPoint.at<double>(2, 0));

	world = realPoint;

//	cout<<"Pixel Point "<<pixel<<" translates to "<<world<<" in world coords"<<endl;

	return true;
}

void ImageProcessor::worldToPixelCoordinates(vector<Point3f> objPoints, vector<Point2f> &imgPoints, Mat _rvec, Mat _tvec, Mat _camMatrix, Mat _distCoeffs) {
	projectPoints(objPoints, _rvec, _tvec, _camMatrix, _distCoeffs, imgPoints);
}

vector<Point2f> ImageProcessor::calcIdealBlockFacesImg(double& pWidth, double& pHeight, double& pDepth, double blockWidth, double blockHeight, double blockDepth) {
	vector<Point3f> objPoints;
	Point3f centre = Point3f(ARUCO_YOFFSET,ARUCO_XOFFSET,0);
	//regular top
	objPoints.push_back(Point3f(centre.x+blockWidth,centre.y,centre.z));
	objPoints.push_back(Point3f(centre.x+blockWidth,centre.y+blockHeight,centre.z));
	objPoints.push_back(Point3f(centre.x,centre.y+blockHeight,centre.z));
	objPoints.push_back(Point3f(centre.x,centre.y,centre.z));
	objPoints.push_back(Point3f(centre.x+blockDepth,centre.y,centre.z));

//	objPoints.push_back(Point3f(centre.x+blockWidth,centre.y,centre.z+blockDepth));
//	objPoints.push_back(Point3f(centre.x+blockWidth,centre.y+blockHeight,centre.z+blockDepth));
//	objPoints.push_back(Point3f(centre.x,centre.y+blockHeight,centre.z+blockDepth));
//	objPoints.push_back(Point3f(centre.x,centre.y,centre.z+blockDepth));

	vector<Point2f> imgPoints;
	worldToPixelCoordinates(objPoints,imgPoints, rvec, tvec, camMatrix, distCoeffs);

	double temp = distanceBetweenTwoPoints(imgPoints[0].x,imgPoints[0].y,imgPoints[1].x,imgPoints[1].y);
	double temp2 = distanceBetweenTwoPoints(imgPoints[1].x,imgPoints[1].y,imgPoints[2].x,imgPoints[2].y);
	pDepth = distanceBetweenTwoPoints(imgPoints[3].x,imgPoints[3].y,imgPoints[4].x,imgPoints[4].y);
	pWidth = min(temp,temp2);
	pHeight = max(temp,temp2);

	cout<<"Dimens of ideal block (px) : "<<pWidth<<"\t\t"<<pHeight<<"\t\t"<<pDepth<<endl;

	return imgPoints;

}

void ImageProcessor::mousePressCallback(int event, int x, int y){
	if(event == EVENT_LBUTTONDOWN) {
		Point2f pixel((float)x,(float)y);
		Point3f world;
#ifdef ARM_CALIBRATION
		calibIdx++;
		if(calibIdx > 4) calibIdx = 0;
#else
		if(pixelToWorldCoordinates(pixel, world)) {
#ifdef OPENCV_CALIB
			cout<<"Pixel Point "<<pixel<<" translates to "<<world<<" in world coords\n"<<endl;
#else
			FullPosition fp(MyPoint(world.x,world.y,BLK_GRB_HEIGHT),Rotation(0,0,0));
			targetPos = fp;
			targetSet = true;
#endif
		}

#endif
	}
}

void ImageProcessor::mousePressCallback(int event, int x, int y, int, void* userData){
	if(event != EVENT_LBUTTONDOWN) {
		return;
	}

	if(!userData) {
		cout<<"Invalid user parameters\n";
		return;
	}
	ImageProcessor* ip = reinterpret_cast<ImageProcessor*>(userData);
	ip->mousePressCallback(event, x, y);

}

void ImageProcessor::trackbarCallback(int, void* userdata){
	if(!userdata) {
		cout<<"Invalid user parameters\n";
		return;
	}
	ImageProcessorCallback* ipc = reinterpret_cast<ImageProcessorCallback*>(userdata);
//	cout<<"passed image size "<<ipc->img.size << endl;
	rectDetect(ipc->rects, ipc->img, ipc->thresh, ipc->imgOut);
//	cout<<"calculated image size "<<ipc->imgOut <<endl;
//	rectDetect(ipc->corners, ipc->img, thresh);

}

void ImageProcessor::trackBarJointAngles(int val, void* userdata) {
	ActuatorNum* a = reinterpret_cast<ActuatorNum*>(userdata);
	switch(*a) {
	case A0:
		tbJointAngles.setAngle(0,val);
		break;
	case A1:
		tbJointAngles.setAngle(1,val);
		break;
	case A2:
		tbJointAngles.setAngle(2,val);
		break;
	case A3:
		tbJointAngles.setAngle(3,val);
		break;
	case A4:
		tbJointAngles.setAngle(4,val);
		break;
	case A5:
		tbJointAngles.setAngle(5,val);
		break;
	case A6:
		tbJointAngles.setAngle(6,val);
		break;
	}
	tbJointAngles.isReady = true;
}

bool ImageProcessor::rectDetect(vector<RotatedRect>& rects, Mat imgSrc, int cannyThresh, Mat& imgOut){
	Mat imgGrey;
    if(!imgSrc.empty()) {
		cvtColor(imgSrc, imgGrey, COLOR_BGR2GRAY);
		blur(imgGrey,imgGrey, Size(3,3));
        Mat cannyOutput;
        Canny(imgGrey, cannyOutput, cannyThresh, cannyThresh*2 );
        vector<vector<Point> > contours;

        findContours(cannyOutput, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        if(contours.empty()) return false;

        vector<RotatedRect> inRects(contours.size());

        for( size_t i = 0; i < contours.size(); i++ ) {
        	inRects[i] = minAreaRect(contours[i]);
        }
        rects = inRects;

        /// Draw contours
        RNG rng(12345);
        Mat drawing = Mat::zeros( cannyOutput.size(), CV_8UC3 );
        for( int i = 0; i< (int)contours.size(); i++ ){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//	        drawContours( drawing, contours, (int)i, color );
	        // rotated rectangle
	        Point2f rect_points[4];
	        inRects[i].points( rect_points );
	        for ( int j = 0; j < 4; j++ ){
	            line( drawing, rect_points[j], rect_points[(j+1)%4], color );
	        }
        }
//        imshow("Contours",drawing);
        waitKey(10);
        return true;

    }
    return false;
}

FullPosition ImageProcessor::rectToRealPos(RectWithType rect) {
	double zOffset;
	switch(rect.type) {
	case 0:
		zOffset = BLOCK_DEPTH*2;
		break;
	case 1:
		zOffset = BLOCK_WIDTH*2;
		break;
	case 2:
		zOffset = BLOCK_LENGTH*2;
		break;
	}
//	zOffset = 150;
	Point3f world;
	pixelToWorldCoordinates(rect.rect.center,world);
//	world.z = zOffset;
	world.z = 150.0;
	FullPosition fp(MyPoint(world.x,world.y,world.z),Rotation(0,0,toRadians(0)));
//	FullPosition fp(MyPoint(world.x,world.y,world.z),Rotation(0,0,toRadians(rect.rect.angle)));


	return fp;
}

bool ImageProcessor::filterBlockDetection(vector<RotatedRect> inRects, RectWithType& idealRect, Mat imgSrc, double pWidth, double pHeight, double pDepth, int tolerance) {
	cout<<flush;
	rectfile<<flush;
	vector<RectWithType> correctRects;
	for(int i=0; i<(int)inRects.size(); ++i) {
		int res = withinBlockDimens(pWidth,pHeight,pDepth,inRects[i].size.width,inRects[i].size.height,tolerance);
		RectWithType temp(inRects[i],res);
		if(res >= 0 && !squareCheck(temp) && minSizeRectCheck(temp.rect,pWidth,pDepth)) {
			rectfile <<i<<"		"<<inRects[i].size.width<<"		"<<inRects[i].size.height<<"		"<<inRects[i].center<<"			"<<res<<"		"<<tolerance<<endl;
			correctRects.push_back(temp);
		}
	}
//	outRects = correctRects;
	Mat drawing = Mat::zeros( imgSrc.size(), CV_8UC3 );
	RNG rng(12345);

	for( size_t i = 0; i< correctRects.size(); i++ ) {
		Scalar colour = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
		Point2f rectPoints[4];
		correctRects[i].rect.points(rectPoints);
		for ( int j = 0; j < 4; j++ ){
			line( drawing, rectPoints[j], rectPoints[(j+1)%4], colour );
		}
	}
	imshow("Filtered rects",drawing);

	return optimalRectSearch(correctRects, idealRect, pWidth, pHeight, pDepth);

}

Mat ImageProcessor::drawTarget(Mat imgSrc, RotatedRect rect) {
//	Mat drawing = Mat::zeros( imgSrc.size(), CV_8UC3 );
	Scalar colour = Scalar(0,0,255);
	Point2f rectPoints[4];
	rect.points(rectPoints);
	for ( int j = 0; j < 4; j++ ){
		line(imgSrc, rectPoints[j], rectPoints[(j+1)%4], colour, 4);
	}
	return imgSrc;
}

Mat ImageProcessor::drawIdealBlock(Mat imgSrc, vector<Point2f> points) {
//	Mat drawing = Mat::zeros( imgSrc.size(), CV_8UC3 );
	Scalar colour = Scalar(255,0,0);
	for (int j = 0; j < 4; j++ ){
		line(imgSrc, points[j], points[(j+1)%4], colour, 1);
	}
	return imgSrc;
}

bool ImageProcessor::withinArucoMarkerCheck(Point3f point, Point3f centre, double markerLength,int tolerance){
	if(inRange(point.x,centre.x-markerLength-tolerance,centre.x+markerLength+tolerance)) {
		rectfile<<"Point "<<point<< "  within aruco x range  "<<centre.x-markerLength-tolerance<<"	-	"<<centre.x+markerLength+tolerance<<endl;
		return true;
	}
	if(inRange(point.y,centre.y-markerLength-tolerance,centre.y+markerLength+tolerance)) {
		rectfile<<"Point "<<point<< "  within aruco y range  "<<centre.y-markerLength-tolerance<<"	-	"<<centre.y+markerLength+tolerance<<endl;
		return true;
	}
	return false;
}

bool ImageProcessor::withinArucoPixelCheck(RotatedRect rect, vector<Point2f> corners){
	double margin = (double)BLOCK_TOLERANCE;
	Point2f pCorners[4];
	rect.points(pCorners);
//	double flag = false;
	for(int i=0; i<4; ++i) {
		pCorners[i].x += pCorners[i].x >= 0? margin : -margin;
		pCorners[i].y += pCorners[i].y >= 0? margin : -margin;

		double res = pointPolygonTest(corners, pCorners[i], false);
		if(res >= 0) {
//			rectfile<<"Point "<<pCorners[i]<< "  within aruco range  "<<endl;
			return true;
		}
	}
	return false;
}

bool ImageProcessor::minSizeRectCheck(RotatedRect temp, double pWidth, double pDepth){
	return temp.size.area() > (pWidth*pDepth)*0.5;
}

bool ImageProcessor::squareCheck(RectWithType rect, int tolerance){
//	if(rect.type != 2) {
		if(fabs(rect.rect.size.height - rect.rect.size.width) < tolerance) {
			return true;
		}
//	}
	return false;
}

bool ImageProcessor::optimalRectSearch(vector<RectWithType> rects, RectWithType& opt, double blockWidth, double blockHeight, double blockDepth) {
	if(!rects.empty()) {
		for(auto it=rects.begin(); it!=rects.end();++it) {
			double tArea;
			switch(it->type){
			case 0:
				tArea = blockWidth * blockHeight;
				break;
			case 1:
				tArea = blockDepth * blockHeight;
				break;
			case 2:
				tArea = blockWidth * blockDepth;
				break;
			}
			it->targetArea = tArea;
		}
		sort(rects.begin(),rects.end(),sortRectNearestArea);
		bool prefSetFlag = false;
		int idx = 0, count=0;
		for(auto it=rects.begin(); it!=rects.end();++it) {
			if(!prefSetFlag) {
				if(it->type == 0) {
					prefSetFlag = true;
					idx = count;
				}
				if(it->type == 1) {
					prefSetFlag = true;
					idx = count;
				}
			}
			count++;
//			cout<<*it;
		}
		opt = rects[idx];
//		cout<<"Winner\n"<<opt<<endl;
		return true;
	}
	return false;

//	RotatedRect target;
//	vector<RectWithType> rectType0, rectType1, rectType2;
//
//	for(auto it = rects.begin(); it != rects.end(); ++it){
//		if(it->type == 0) {
//			rectType0.push_back(*it);
//		}
//		if(it->type == 1) {
//			rectType1.push_back(*it);
//		}
//		if(it->type == 2) {
//			rectType2.push_back(*it);
//		}
//	}
//
//	RectWithType ref0(RotatedRect(Point2f(0,0),Size2f(blockWidth,blockHeight),0),0);
//	RectWithType ref1(RotatedRect(Point2f(0,0),Size2f(blockHeight,blockDepth),0),1);
//	RectWithType ref2(RotatedRect(Point2f(0,0),Size2f(blockWidth,blockDepth),0),2);
//
//    auto const it0 = lower_bound(rectType0.begin(), rectType0.end(), ref0,
//    		[](RectWithType lhs, RectWithType rhs) -> bool { return lhs.rect.size.area() < rhs.rect.size.area(); });
////    auto const it1 = lower_bound(rectType1.begin(), rectType1.end(), ref1,
////			[](RectWithType lhs, RectWithType rhs) -> bool { return lhs.rect.size.area() < rhs.rect.size.area(); });
////    auto const it2 = lower_bound(rectType2.begin(), rectType2.end(), ref2,
////    		[](RectWithType lhs, RectWithType rhs) -> bool { return lhs.rect.size.area() < rhs.rect.size.area(); });
////
//    return it0 != rectType0.end()? *it0 : it1 != rectType1.end()? *it1 : it2 != rectType2.end()? *it2 : rects[rects.size()/2];

}

//bool ImageProcessor::maxBlockErrorThresh(RectWithType rect, int thresh){
//
//}

bool ImageProcessor::sortRectVecAscArea(RectWithType lhs, RectWithType rhs) {
	return lhs.rect.size.area() < rhs.rect.size.area();
}

bool ImageProcessor::sortRectsByType(RectWithType lhs, RectWithType rhs){
	return lhs.type < rhs.type;
}

bool ImageProcessor::sortRectNearestArea(RectWithType lhs, RectWithType rhs) {
//	if(lhs.type < rhs.type) return false;

	double lDiff = fabs(lhs.rect.size.area() - lhs.targetArea);
	double rDiff = fabs(rhs.rect.size.area() - rhs.targetArea);

	return lDiff < rDiff;

}

void ImageProcessor::targetSetDelay(unsigned int delayms){
	int del1 = delayms * 0.75;
	int del2 = delayms = del1;
	this_thread::sleep_for(chrono::milliseconds(del1));
	homeMoveFlag = true;
	this_thread::sleep_for(chrono::milliseconds(del2));
	readyForNewImgFlag = true;
}

bool ImageProcessor::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
     FileStorage fs(filename, FileStorage::READ);
     if(!fs.isOpened())
         return false;
     fs["camera_matrix"] >> camMatrix;
     fs["distortion_coefficients"] >> distCoeffs;
     return true;
 }

bool ImageProcessor::readCalibTargets(vector<FullPosition>& targets, string fileName){
    ifstream file(fileName);
    if(!file.is_open())
        return false;
    FullPosition fp;
    while(file >> fp.position.x >> fp.position.y >> fp.position.z >> fp.orientation.x >> fp.orientation.y >> fp.orientation.z) {
//    	fp.closeGripper();
    	targets.push_back(fp);
    }
    return targets.size()>0? true:false;
}

bool ImageProcessor::userAction(MovementController& mc){
	rectfile.open("res/rectangles.txt");
	rectfile<<"idx		width		height		result		"<<endl;
    bool readOk = camMatSet();
    if(!readOk) {
        cerr << "Invalid camera setup" << endl;
        return false;
    }

    namedWindow("Workspace");
    setMouseCallback("Workspace", mousePressCallback, this);
    ImageProcessorCallback userData;

#ifndef ARM_CALIBRATION
    if(!ready()) calcPositionByAruco();
    cout<<flush;
    int maxThresh = 255;
    int tempJALower = MG996R_MAX_ANGLE, tempJAUpper = MG90S_MAX_ANGLE, gripperLower = 0;
    createTrackbar("Canny Thresh:","Workspace",&userData.thresh, maxThresh, trackbarCallback, &userData);
    createTrackbar("J0:","Workspace",&tempJALower, tempJALower*2, trackBarJointAngles, new ActuatorNum(A0));
    createTrackbar("J1:","Workspace",&tempJALower, tempJALower*2, trackBarJointAngles, new ActuatorNum(A1));
    createTrackbar("J2:","Workspace",&tempJALower, tempJALower*2, trackBarJointAngles, new ActuatorNum(A2));
    createTrackbar("J3:","Workspace",&tempJAUpper, tempJAUpper*2, trackBarJointAngles, new ActuatorNum(A3));
    createTrackbar("J4:","Workspace",&tempJAUpper, tempJAUpper*2, trackBarJointAngles, new ActuatorNum(A4));
    createTrackbar("J5:","Workspace",&tempJAUpper, tempJAUpper*2, trackBarJointAngles, new ActuatorNum(A5));
    createTrackbar("J6:","Workspace",&gripperLower, 1, trackBarJointAngles, new ActuatorNum(A6));

    double idealWidth, idealHeight, idealDepth;
    vector<Point2f> idealBlockFaces = calcIdealBlockFacesImg(idealWidth, idealHeight, idealDepth);
//    mc.moveHome();
//	this_thread::sleep_for(chrono::milliseconds(3000));

	autoMode = false;
#ifdef TRGT_MOVE
	autoMode = true;
#endif

#else
	if(!readCalibTargets(calibTargets)) {
		cout<<"Invalid target ARM_CALIBRATION file\n";
		return false;
	}
	int currentCalibIdx = -1;
	calibIdx = 0;
	int loopCounter = -1;
#endif

    while(cap->grab()) {
    	cout<<flush;
    	tbJointAngles.isReady = false;
    	Mat distFrame, frame;
    	cap->retrieve(distFrame);
    	undistort(distFrame,frame,camMatrix,distCoeffs);

        userData.img = frame;
        imshow("Workspace", frame);

#ifndef ARM_CALIBRATION
        trackbarCallback(userData.thresh, &userData);

        if(autoMode){
            if(!userData.rects.empty() && readyForNewImgFlag) {
                RectWithType targetRect;
                if(filterBlockDetection(userData.rects,targetRect,frame, idealWidth, idealHeight, idealDepth, BLOCK_TOLERANCE)){
            		setReadyFlag(false);
    				imshow("Target", drawTarget(frame,targetRect.rect));
    				FullPosition targetPos = rectToRealPos(targetRect);
    				thread sleepThread(&ImageProcessor::targetSetDelay, this, TRGT_MV_DLY);
    				sleepThread.detach();
    				if(mc.setTargetPosition(targetPos)) {
    					this_thread::sleep_for(chrono::milliseconds(1000));
    					mc.initBlockGrab(targetPos);
    				}
    //				break;
                }
            }

            if(homeMoveFlag) {
            	homeMoveFlag = !mc.moveHome();
            }
        }
//        if(userData.imgOut.data) imshow("Edges", userData.imgOut);


        if(tbJointAngles.isReady){
        	targetPos = FullPosition(JointAngles(tbJointAngles.angles));
        	targetSet = true;
        }

        if(targetSet) {
        	if(mc.setTargetPosition(targetPos)) {
//        		cout<<"Target success"<<endl;
//        		mc.initBlockGrab(targetPos);
        	}
        	targetSet = false;
        }
#else
        if(currentCalibIdx != calibIdx) {
        	cout<<"Setting position \t"<<calibIdx<<"\t"<<calibTargets[calibIdx]<<endl;
        	if(mc.setTargetPosition(calibTargets[calibIdx]))
        		cout<<"Target successful\n";
        	else
        		cout<<"Target unsuccessful\n";
        	currentCalibIdx = calibIdx;
        	if(calibIdx == 0) loopCounter++;
        	cout<<"Iteration no \t"<<loopCounter<<endl;
        }


#endif

        char key = (char)waitKey(10);
        if(key == 113) {
        	autoMode = !autoMode;
        	cout<<"CHanging mode \t"<<autoMode;
        }
        if(key == 27) break;
    }
    return true;
}

ostream& operator<<(ostream& os, const RectWithType& r) {
	double area = fabs(r.targetArea-r.rect.size.area());
	cout<<flush;
    os << "Type : "<<r.type <<"\t Area: "<<r.rect.size.area()<<"\t\tTargetArea: "<<r.targetArea<<"\t\tActual Area<<"<<r.rect.size.area()<<"\t\tarea diff: "<<area<<endl;
    return os;
}


