//============================================================================
// Lecture      : PHOTOGRAMMETRIC COMPUTER VISION
// Exercise No. : 1
// Name         : Pcv1.cpp
// Author       : Ronny Haensch
// Edited by    : Group L
// Group member : Armin Forouharfard
//	              Nicolas Jakob Wagener 
//				  Arghadeep Mazumder
//                Sandeep Kumar Sahoo 
// Date        :  Winter semester 2016/17
// Version     :  1.0
// Copyright   :  -
// Description : 
//============================================================================

#include "Pcv2.h"

// compute the homography
/*
base		first set of points x'
attach		second set of points x
return		homography H, so that x' = Hx
*/
Mat Pcv2::homography2D(Mat& base, Mat& attach){
	// TO DO !!!@
	Mat T_base = Pcv2::getCondition2D(base);
	Mat T_attach = Pcv2::getCondition2D(attach);
	Mat base_con = T_base*base;
	Mat attach_con = T_attach*attach;
	Mat A = getDesignMatrix_homography2D(base_con, attach_con);
	Mat H = Pcv2::solve_dlt(A);
	decondition(T_base, T_attach, H);
	return H;
}

// solve homogeneous equation system by usage of SVD
/*
A		the design matrix
return	solution of the homogeneous equation system
*/

Mat Pcv2::solve_dlt(Mat& A){
	// TO DO !!!@
	Mat H2 = Mat(3, 3, CV_32FC1);
	cv::SVD svd(A, SVD::FULL_UV);
	svd.vt.row(8).colRange(0, 3).copyTo(H2.row(0));
	svd.vt.row(8).colRange(3, 6).copyTo(H2.row(1));
	svd.vt.row(8).colRange(6, 9).copyTo(H2.row(2));
	return H2;
}

// decondition a homography that was estimated from conditioned point clouds
/*
T_base		conditioning matrix T' of first set of points x'
T_attach	conditioning matrix T of second set of points x
H			conditioned homography that has to be un-conditioned (in-place)
*/
void Pcv2::decondition(Mat& T_base, Mat& T_attach, Mat& H){
  	// TO DO !!!@
	H = T_base.inv() * H * T_attach;
}

// define the design matrix as needed to compute 2D-homography
/*
base	first set of points x' --> x' = H * x
attach	second set of points x --> x' = H * x
return	the design matrix to be computed
*/
Mat Pcv2::getDesignMatrix_homography2D(Mat& base, Mat& attach){
	// TO DO !!!@
	int i, j;
	Mat A = Mat::zeros(2 * base.cols, 9, CV_32FC1);
	for (i = 0; i < base.cols; i++) {
		for (j = 0; j < base.rows; j++) {
			A.at<float>(i * 2, j) = -1 * base.at<float>(2, i) * attach.at<float>(j, i);
			A.at<float>(i * 2, j + 6) = base.at<float>(0, i) * attach.at<float>(j, i);
			A.at<float>(i * 2 + 1, j + 3) = -1 * base.at<float>(2, i) * attach.at<float>(j, i);
			A.at<float>(i * 2 + 1, j + 6) = base.at<float>(1, i) * attach.at<float>(j, i);
		}
	}
	return A;
}

// apply transformation to set of points
/*
H			matrix representing the transformation
geomObj		matrix with input objects (one per column)
type		the type of the geometric object (for now: only point and line)
return		transformed objects (one per column)
*/
Mat Pcv2::applyH(Mat& geomObj, Mat& H, string type){

  // if object is a point
  if (type.compare("point") == 0){
      	// TO DO !!!@
		return H*geomObj;
  }
  // if object is a line
  if (type.compare("line") == 0){
      	// TO DO !!!@
		return H.inv().t()*geomObj;
  }
  cout << "ERROR: Do not know how to move " << type << endl;
  cout << "Returning original object" << endl;
  cout << "Press enter key to continue..." << endl;
  cin.get();
  return geomObj;
}

// get the conditioning matrix of given points
/*
p		the points as matrix
return	the condition matrix (already allocated)
*/
Mat Pcv2::getCondition2D(Mat& p){
	// TO DO !!!@
	Mat one = Mat::ones(1, p.cols, CV_32FC1);

	double ty = p.row(1).dot(one) / p.cols;
	double tx = p.row(0).dot(one) / p.cols;

	double sy = (abs(ty*one - p.row(1))).dot(one) / p.cols;
	double sx = (abs(tx*one - p.row(0))).dot(one) / p.cols;
	
	Mat T = Mat::eye(3, 3, CV_32S);
	if (sx != 0 && sy != 0) {
		T = (Mat_<float>(3, 3) << 1 / sx, 0, -tx / sx, 0, 1 / sy, -ty / sy, 0, 0, 1);
	}
	return T;
}

/* *****************************
  GIVEN FUNCTIONS
***************************** */

// stitches two images together by transforming one of them by a given homography
/*
base		the base image
attach		the image to be attached
H		the homography to warp the second image
panorama	the resulting image
*/
Mat Pcv2::stitch(Mat& base, Mat& attach, Mat& H){

    // compute corners of warped image
    Mat corners(1, 4, CV_32FC2);
    corners.at<Vec2f>(0, 0) = Vec2f(0,0);
    corners.at<Vec2f>(0, 1) = Vec2f(0,attach.rows);
    corners.at<Vec2f>(0, 2) = Vec2f(attach.cols,0);
    corners.at<Vec2f>(0, 3) = Vec2f(attach.cols,attach.rows);
    perspectiveTransform(corners, corners, H);
    
    // compute size of resulting image and allocate memory
    float x_start = min( min( corners.at<Vec2f>(0, 0)[0], corners.at<Vec2f>(0, 1)[0]), (float)0);
    float x_end   = max( max( corners.at<Vec2f>(0, 2)[0], corners.at<Vec2f>(0, 3)[0]), (float)base.cols);
    float y_start = min( min( corners.at<Vec2f>(0, 0)[1], corners.at<Vec2f>(0, 2)[1]), (float)0);
    float y_end   = max( max( corners.at<Vec2f>(0, 1)[1], corners.at<Vec2f>(0, 3)[1]), (float)base.rows);

    // create translation matrix in order to copy both images to correct places
    Mat T = Mat::zeros(3,3,CV_32FC1);
    T.at<float>(0, 0) = 1;
    T.at<float>(1, 1) = 1;
    T.at<float>(2, 2) = 1;
    T.at<float>(0, 2) = -x_start;
    T.at<float>(1, 2) = -y_start;
  
    // change homography to take necessary translation into account
    T = T * H;
    // warp second image and copy it to output image
    Mat panorama;
    warpPerspective(attach, panorama, T, Size(x_end - x_start + 1, y_end - y_start + 1), CV_INTER_LINEAR);

    // copy base image to correct position within output image
    Mat roi(panorama, Rect(-x_start,-y_start,base.cols, base.rows));
    base.copyTo(roi, base);
  
    return panorama;
 
}

// mouse call back to get points and draw circles
/*
event	specifies encountered mouse event
x,y	position of mouse pointer
flags	not used here
param	a struct containing used IplImage and window title
*/
void getPointsCB(int event, int x, int y, int flags, void* param){

  // cast to a structure
  struct winInfo* win = (struct winInfo*)param;
  
  switch(event){
    // if left mouse button was pressed
    case CV_EVENT_LBUTTONDOWN:{
		// create point representing mouse position
		Point2f p = Point2f(x,y);
		// draw green point
		circle(win->img, p, 2, Scalar(0, 255, 0), 2);
		// draw green circle
		circle(win->img, p, 15, Scalar(0, 255, 0), 2);
		// update image
		imshow(win->name.c_str(), win->img);
		// add point to point list
		win->pointList.push_back(p);
    }break;
  }
}

// display two images and catch the point pairs marked by left mouse clicks
// points will be in homogeneous coordinates
/*
base		structure containing base image
attach		structure containing image that has to be attached
p_base		points within the base image (to be defined by this method)
p_attach	points within the second image (to be defined by this method)
*/
int Pcv2::getPoints(struct winInfo& base, struct winInfo& attach, Mat& p_base, Mat& p_attach){
 
    cout << endl;
    cout << "Please select at least four points by clicking at the corresponding image positions:" << endl;
    cout << "Firstly click at the point that shall be transformed (within the image to be attached), followed by a click on the corresponding point within the base image" << endl;
    cout << "Continue until you have collected as many point pairs as you wish" << endl;
    cout << "Stop the point selection by pressing any key" << endl << endl;
  
    // show input images and install mouse callback
    namedWindow( base.name.c_str(), 0 );
    imshow( base.name.c_str(), base.img );
    base.pointList.clear();
    setMouseCallback(base.name.c_str(), getPointsCB, (void*) &base);
    namedWindow( attach.name.c_str(), 0 );
    imshow( attach.name.c_str(), attach.img );
    attach.pointList.clear();
    setMouseCallback(attach.name.c_str(), getPointsCB, (void*) &attach);
    // wait until any key was pressed
    waitKey(0);
    
    destroyWindow( base.name.c_str() );
    destroyWindow( attach.name.c_str() );

    // allocate memory for point-lists (represented as matrix)
    int numOfPoints = base.pointList.size();
    p_base = Mat(3, numOfPoints, CV_32FC1);
    p_attach = Mat(3, numOfPoints, CV_32FC1);
    // read points from global variable, transform them into homogeneous coordinates
    for(int p = 0; p<numOfPoints; p++){
		p_attach.at<float>(0, p) = attach.pointList.at(p).x;
		p_attach.at<float>(1, p) = attach.pointList.at(p).y;
		p_attach.at<float>(2, p) = 1;
		p_base.at<float>(0, p) = base.pointList.at(p).x;
		p_base.at<float>(1, p) = base.pointList.at(p).y;
		p_base.at<float>(2, p) = 1;
    }
    return numOfPoints;

}

// function loads input image, calls processing function, and saves result
/*
fname	path to input image
*/
void Pcv2::run(string fnameBase, string fnameLeft, string fnameRight){

    // titles of some windows
    string winName1 = string("Base image");
    string winName2 = string("Image to attach");
    string winName3 = string("Panorama");

    // load image first two images, paths in argv[1] and argv[2]
    Mat baseImage = imread(fnameBase);
    Mat attachImage = imread(fnameLeft);
    if (!baseImage.data){
		cerr << "ERROR: Cannot read image ( " << fnameBase << endl;
		cin.get();
		exit(-1);
	}
	if (!attachImage.data){
		cerr << "ERROR: Cannot read image ( " << fnameLeft << endl;
		cin.get();
		exit(-1);
	}
	namedWindow("Imagebase");
	imshow("Imagebase", baseImage);
	
	namedWindow("Imageattach");
	imshow("Imageattach", attachImage);

	waitKey(0);
	destroyWindow("Imagebase");
	destroyWindow("Imageattach");

    // fuse image data and window title
    struct winInfo base;
    base.img = baseImage.clone();
    base.name = winName1;    
    struct winInfo attach;
    attach.img = attachImage.clone();
    attach.name = winName2;
   
    // get corresponding points within the two image
    // start with one point within the attached image, then click on corresponding point in base image
    Mat p_basis, p_attach;
    int numberOfPointPairs = getPoints(base, attach, p_basis, p_attach);
    
    // just some putput
    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in base image:" << endl;
    cout << p_basis << endl;
    cout << endl << "Points in second image:" << endl;
    cout << p_attach << endl;

    // calculate homography
    Mat H = homography2D(p_basis, p_attach);
    
    // create panorama
    Mat panorama = stitch(baseImage, attachImage, H);

    // display panorama (resizeable)
    namedWindow( winName3.c_str(), 0 );
    imshow( winName3.c_str(), panorama );
    waitKey(0);
    destroyWindow( winName3.c_str());
    
    // panorama is new base image, third image is the image to attach
    baseImage = panorama;
    // load third image
    attachImage = imread(fnameRight);
    if (!attachImage.data){
		cout << "ERROR: Cannot read image ( " << fnameRight << " )" << endl;
		cin.get();
		exit(-1);
	}
    
    // fuse image data and window title
    base.img = baseImage.clone();
    attach.img = attachImage.clone();
    
    // get corresponding points within the two image
    // start with one point within the attached image, then click on corresponding point in base image
    numberOfPointPairs = getPoints(base, attach, p_basis, p_attach);
    
    // just some putput
    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in base image:" << endl;
    cout << p_basis << endl;
    cout << endl << "Points in second image:" << endl;
    cout << p_attach << endl;
    
    // calculate homography
    H = homography2D(p_basis, p_attach);
    
    // create panorama
    panorama = stitch(baseImage, attachImage, H);
     
    // display panorama (resizeable)
    namedWindow( winName3.c_str(), 0 );
    imshow( winName3.c_str(), panorama );
    waitKey(0);
    destroyWindow( winName3.c_str());
    
    imwrite("panorama.png", panorama);

}

// function calls processing functions
// output is tested on "correctness" 
void Pcv2::test(void){
	test_homography2D();
	test_getCondition2D();
	test_getDesignMatrix_homography2D();
	test_solve_dlt();
	test_decondition();
}

void Pcv2::test_getCondition2D(void){

	Mat p = (Mat_<float>(3,4) << 93, 729, 703, 152, 617, 742, 1233, 1103, 1, 1, 1, 1);
	Mat Ttrue = (Mat_<float>(3,3) << 1./296.75, 0, -419.25/296.75, 0, 1./244.25, -923.75/244.25, 0, 0, 1);
	
	Mat Test = getCondition2D(p);
	if ( (Test.rows != 3) || (Test.cols != 3) || (Test.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::getCondition2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Test.convertTo(Test, CV_32FC1);
	Test = Test / Test.at<float>(2,2);
	float eps = pow(10,-3);
	if (sum(abs(Test - Ttrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::getCondition2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

void Pcv2::test_getDesignMatrix_homography2D(void){
	
	Mat p1 = (Mat_<float>(3,4) << -1, 1, 1, -1, -1, -1, 1, 1,  1, 1, 1, 1);
	Mat p2 = (Mat_<float>(3,4) << -1.0994103, 1.0438079, 0.9561919, -0.90058976,  -1.2558856, -0.74411488, 1.2661204, 0.73387909, 1, 1, 1, 1);
	
	Mat Aest = getDesignMatrix_homography2D(p1, p2);
	if ( ( (Aest.rows != 8) && (Aest.rows != 9) ) || (Aest.cols != 9) || (Aest.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::getDesignMatrix_homography2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Aest.convertTo(Aest, CV_32FC1);
	Mat Atrue;
	if (Aest.rows == 8)
		Atrue = (Mat_<float>(8,9) << 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 1.0994103, 1.2558856, -1, -1.0438079, 0.74411488, -1, 0, 0, 0, 1.0438079, -0.74411488, 1, 0, 0, 0, -1.0438079, 0.74411488, -1, -1.0438079, 0.74411488, -1, -0.9561919, -1.2661204, -1, 0, 0, 0, 0.9561919, 1.2661204, 1, 0, 0, 0, -0.9561919, -1.2661204, -1, 0.9561919, 1.2661204, 1, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, -0.90058976, 0.73387909, 1);
	else
		Atrue = (Mat_<float>(9,9) << 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 1.0994103, 1.2558856, -1, -1.0438079, 0.74411488, -1, 0, 0, 0, 1.0438079, -0.74411488, 1, 0, 0, 0, -1.0438079, 0.74411488, -1, -1.0438079, 0.74411488, -1, -0.9561919, -1.2661204, -1, 0, 0, 0, 0.9561919, 1.2661204, 1, 0, 0, 0, -0.9561919, -1.2661204, -1, 0.9561919, 1.2661204, 1, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, -0.90058976, 0.73387909, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	float eps = pow(10,-3);
	if (sum(abs(Aest - Atrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::getDesignMatrix_homography2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
		
}

void Pcv2::test_solve_dlt(void){
	Mat A = (Mat_<float>(8,9) << 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 1.0994103, 1.2558856, -1, -1.0438079, 0.74411488, -1, 0, 0, 0, 1.0438079, -0.74411488, 1, 0, 0, 0, -1.0438079, 0.74411488, -1, -1.0438079, 0.74411488, -1, -0.9561919, -1.2661204, -1, 0, 0, 0, 0.9561919, 1.2661204, 1, 0, 0, 0, -0.9561919, -1.2661204, -1, 0.9561919, 1.2661204, 1, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, -0.90058976, 0.73387909, 1);
	Mat Hest = solve_dlt(A);
	if ( (Hest.rows != 3) || (Hest.cols != 3) || (Hest.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::solve_dlt(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Hest.convertTo(Hest, CV_32FC1);
	Hest = Hest / Hest.at<float>(2,2);
	Mat Htrue = (Mat_<float>(3,3) << 0.57111752, -0.017852778, 0.013727478, -0.15091757, 0.57065326, -0.04098846, 0.024604173, -0.041672569, 0.56645769);
	Htrue = Htrue / Htrue.at<float>(2,2);
	float eps = pow(10,-3);
	if (sum(abs(Hest - Htrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::solve_dlt(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

void Pcv2::test_decondition(void){
	
	Mat H = (Mat_<float>(3,3) << 0.57111752, -0.017852778, 0.013727478, -0.15091757, 0.57065326, -0.04098846, 0.024604173, -0.041672569, 0.56645769);
	Mat T1 = (Mat_<float>(3,3) << 1./319.5, 0, -1, 0, 1./319.5, -1, 0, 0, 1);
	Mat T2 = (Mat_<float>(3,3) << 1./296.75, 0, -419.25/296.75, 0, 1./244.25, -923.75/244.25, 0, 0, 1);
	decondition(T1, T2, H);
	if ( (H.rows != 3) || (H.cols != 3) || (H.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::decondition(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	H.convertTo(H, CV_32FC1);
	H = H / H.at<float>(2,2);
	Mat Htrue = (Mat_<float>(3,3) << 0.9304952, -0.11296108, -16.839279, -0.19729686, 1.003845, -601.02362, 0.00012028422, -0.00024751772, 1);
	float eps = pow(10,-3);
	if (sum(abs(H - Htrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::decondition(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

void Pcv2::test_homography2D(void){

	Mat p1 = (Mat_<float>(3,4) << 0, 639, 639, 0, 0, 0, 639, 639, 1, 1, 1, 1);	
	Mat p2 = (Mat_<float>(3,4) << 93, 729, 703, 152, 617, 742, 1233, 1103, 1, 1, 1, 1);
		
	Mat Hest = homography2D(p1, p2);
	if ( (Hest.rows != 3) || (Hest.cols != 3) || (Hest.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::homography2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Hest.convertTo(Hest, CV_32FC1);
	Hest = Hest / Hest.at<float>(2,2);
	Mat Htrue = (Mat_<float>(3,3) << 0.9304952, -0.11296108, -16.839279, -0.19729686, 1.003845, -601.02362, 0.00012028422, -0.00024751772, 1);
	float eps = pow(10,-3);
	if (sum(abs(Hest - Htrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::homography2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}
