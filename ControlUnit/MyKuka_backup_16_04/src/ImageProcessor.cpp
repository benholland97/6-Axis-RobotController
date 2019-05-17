/*
 * ImageProcessor.cpp
 *
 *  Created on: 7 Apr 2019
 *      Author: Ben
 */

#include "ImageProcessor.h"


ImageProcessor::ImageProcessor(string path) {
	if(multiCamConnectAttempt()) {
		setCamMat(path);
	}
}

ImageProcessor::ImageProcessor(int camIdx, string path) {
	if(connectCam(camIdx)) {
		setCamMat(path);
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
    	cap->retrieve(image);

//    	undistort(distImage,image,camMatrix,distCoeffs);
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
    		calcExtrinsicParamsAruco(corners[0],rvec, tvec, rotMatrix, Point3f(ARUCO_YOFFSET,ARUCO_XOFFSET,0));

//            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rVecs, tVecs);
//            vector<Point3f> cornersWorld = getCornersInCameraWorld(markerLength, rVecs[0], tVecs[0]);
//            for(int i=0; i<(int)cornersWorld.size(); ++i) {
//            	cout<<"Corner "<<i<<" "<<cornersWorld[i]<<endl;
//            }
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

	objPoints.push_back(Point3f(centre.x-halfLength,centre.y+halfLength,centre.z));
	objPoints.push_back(Point3f(centre.x+halfLength,centre.y+halfLength,centre.z));
	objPoints.push_back(Point3f(centre.x+halfLength,centre.y-halfLength,centre.z));
	objPoints.push_back(Point3f(centre.x-halfLength,centre.y-halfLength,centre.z));

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

bool ImageProcessor::pixelToWorldCoordinates(Point2f pixel, Point3f &world, float z){
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


bool ImageProcessor::userAction(){
	rectfile.open("res/rectangles.txt");
	rectfile<<"idx		width		height		result		"<<endl;

    bool readOk = camMatSet();
    if(!readOk) {
        cerr << "Invalid camera setup" << endl;
        return false;
    }
    cout<<flush;
    if(!ready()) calcPositionByAruco();

    namedWindow("Workspace");
    setMouseCallback("Workspace", mousePressCallback, this);
    ImageProcessorCallback userData;
    vector<RotatedRect> targetRects;
    int maxThresh = 255;
    createTrackbar("Canny Thresh:","Workspace",&userData.thresh, maxThresh, trackbarCallback, &userData);
//    createTrackbar("Tolerance:","Workspace",&sliderTol, maxTolerance, trackbarTolerance);

    cout<<"Starting user mouse action"<<endl;
    while(cap->grab()) {
    	cout<<flush;
    	Mat distFrame, frame;
    	cap->retrieve(distFrame);
    	undistort(distFrame,frame,camMatrix,distCoeffs);

        userData.img = frame;
        imshow("Workspace", frame);

        trackbarCallback(userData.thresh, &userData);

        if(!userData.rects.empty()) filterBlockDetection(userData.rects, targetRects, frame, userData.imgOut, BLOCK_TOLERANCE);

        if(userData.imgOut.data) imshow("Edges", userData.imgOut);

        char key = (char)waitKey(10);
        if(key == 27) break;
    }
    return true;
}

void ImageProcessor::mousePressCallback(int event, int x, int y){

	if(event == EVENT_LBUTTONDOWN) {
		Point2f pixel((float)x,(float)y);
		Point3f world;
		if(pixelToWorldCoordinates(pixel, world)) {
			cout<<"Pixel Point "<<pixel<<" translates to "<<world<<" in world coords"<<endl;
		}
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

//void ImageProcessor::trackbarTolerance(int, void* userdata){
//	tol = sliderTol;
//}


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

        return true;

    }
    return false;
}

void ImageProcessor::filterBlockDetection(vector<RotatedRect> inRects, vector<RotatedRect>& outRects, Mat imgSrc, Mat& imgOut, double blockWidth, double blockHeight, double blockDepth, int tolerance) {
	for(int i=0; i<(int)inRects.size(); ++i) {
		int res;
		if(withinArucoPixelCheck(inRects[i],calibArucoCorners)){
			res = -9;
		} else {
			Point2f pCorners[4];
			inRects[i].points(pCorners);
			array<Point3f,4> wCorners;
			for(int j=0; j<4; ++j) {
				pixelToWorldCoordinates(pCorners[j],wCorners[j]);
	//			arucoCheck = withinArucoMarkerCheck(wCorners[j]);
			}
			double temp = distanceBetweenTwoPoints(wCorners[0].x,wCorners[0].y,wCorners[1].x,wCorners[1].y);
			double temp2 = distanceBetweenTwoPoints(wCorners[1].x,wCorners[1].y,wCorners[2].x,wCorners[2].y);

	//		cout<<"Distance 1 "<<temp <<" \tDistance 2: "<<temp2<<endl;
			double wWidth = min(temp,temp2);
			double wHeight = max(temp,temp2);

			int res = withinBlockDimens(blockWidth,blockHeight,blockDepth,wWidth,wHeight,tolerance);

			if(res>0) {
				rectfile <<i<<"		"<<wWidth<<"		"<<wHeight<<"		"<<inRects[i].center<<"			"<<res<<"		"<<tolerance<<endl;

			}


		}

		if(res<0) {
//			cout<<"Invalid rectangle - removing index "<<i<<endl;
			inRects.erase(inRects.begin()+i);
			--i;
		}
	}
	outRects = inRects;

	Mat drawing = Mat::zeros( imgSrc.size(), CV_8UC3 );
	RNG rng(12345);

//	cout<<"rect vect size "<<inRects.size()<<endl;
	for( size_t i = 0; i< inRects.size(); i++ ) {
		Scalar colour = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
		Point2f rectPoints[4];
		inRects[i].points(rectPoints);
//            cout<<"Rectangle found centre :"<<inRects[i].center<<"\n";
		for ( int j = 0; j < 4; j++ ){
			line( drawing, rectPoints[j], rectPoints[(j+1)%4], colour );
		}
	}
	imgOut = drawing;
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
	double halfTol = (double)BLOCK_TOLERANCE*0.5;
	Point2f pCorners[4];
	rect.points(pCorners);
//	double flag = false;
	for(int i=0; i<4; ++i) {
		pCorners[i].x += pCorners[i].x >= 0? halfTol : -halfTol;
		pCorners[i].y += pCorners[i].y >= 0? halfTol : -halfTol;

		double res = pointPolygonTest(corners, pCorners[i], false);
		if(res >= 0) {
//			rectfile<<"Point "<<pCorners[i]<< "  within aruco range  "<<endl;
			return true;
		}
	}
	return false;
}





bool ImageProcessor::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
     FileStorage fs(filename, FileStorage::READ);
     if(!fs.isOpened())
         return false;
     fs["camera_matrix"] >> camMatrix;
     fs["distortion_coefficients"] >> distCoeffs;
     return true;
 }



