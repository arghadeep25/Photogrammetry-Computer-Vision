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

#include "Pcv1.h"
#include<math.h>
using namespace cv;



// calculates joining line between two points
/*
  p1, p2 : the two points in homogeneous coordinates
  return : the joining line in homogeneous coordinates
*/
Mat Pcv1::getConnectingLine(Mat& p1, Mat& p2){
  return p1.cross(p2);
}

// generates translation matrix T defined by translation (dx, dy)^T
/*
  dx, dy : the translation in x- and y-direction, respectively
  return : the resulting translation matrix
*/
Mat Pcv1::getTranslMatrix(double dx, double dy){
	Mat T = (Mat_<float>(3,3)<<1,0,dx, 0,1,dy, 0,0,1);
  return T;
}

// generates rotation matrix R defined by angle phi
/*
  phi		: the rotation angle in degree (!)
  return	: the resulting rotation matrix
*/
Mat Pcv1::getRotMatrix(double phi){
	phi = phi * (3.14159265358) / 180;
	Mat R = (Mat_<float>(3, 3) << cos(phi),-sin(phi),0,    sin(phi),cos(phi),0,    0, 0, 1);
  return R;
}

// generates scaling matrix S defined by scaling factor lambda
/*
  lambda	: the scaling parameter
  return	: the resulting scaling matrix
*/
Mat Pcv1::getScaleMatrix(double lambda){
	Mat S = (Mat_<float>(3, 3) << lambda, 0, 0,    0, lambda,0,    0, 0, 1);
	return S;
}

// combines translation-, rotation-, and scaling-matrices to a single transformation matrix H
/*
  T			: translation matrix
  R			: rotation matrix
  S			: scaling matrix
  return	: resulting homography
*/
Mat Pcv1::getH(Mat& T, Mat& R, Mat& S){
  return S*R*T;
}

// transforms a geometric object by a given homography
/*
  geomObj	: the geometric object to be transformed
  H			: the homography defining the transformation
  type		: the type of the geometric object (for now: only point and line)
  return	: the transformed object
*/
Mat Pcv1::applyH(Mat& geomObj, Mat& H, string type){
  
  // if object is a point
  if (type.compare("point") == 0){
    return H*geomObj;
  }
  // if object is a line
  if (type.compare("line") == 0){
	  return H.inv().t()*geomObj;
  }
  cout << "ERROR: Do not know how to move " << type << endl;
    
}

// checks if a point is on a line
/*
  point		: the given point
  line		: the given line
  eps		: the used accuracy (set to 10^-5 by default (see header))
  return	: true if point is on line
*/
bool Pcv1::isPointOnLine(Mat& point, Mat& line, double eps){
	bool bl = false;
		if (std::abs(point.dot(line)) < eps)
			bl = true;
	return bl;
}

/* *****************************
  GIVEN FUNCTIONS
***************************** */

// function loads input image, calls processing function, and saves result
/*
fname	path to input image
*/
void Pcv1::run(string fname){

    // window names
    string win1 = string ("Image");
  
    // load image as gray-scale, path in argv[1]
    cout << "Load image: start" << endl;
	Mat inputImage = imread(fname, CV_LOAD_IMAGE_GRAYSCALE);
    if (!inputImage.data){
		cout << "ERROR: image could not be loaded from " << fname << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
    }else
		cout << "Load image: done ( " << inputImage.rows << " x " << inputImage.cols << " )" << endl;
	
    // show input image
    namedWindow( win1.c_str(), CV_WINDOW_AUTOSIZE );
    imshow( win1.c_str(), inputImage );
    waitKey(0);

    // the two given points as OpenCV matrices
    Mat x(2, 1, CV_32FC1);
    x.at<float>(0, 0) = 2;
    x.at<float>(1, 0) = 3;
    Mat y(2, 1, CV_32FC1);
    y.at<float>(0, 0) = -4;
    y.at<float>(1, 0) = 5;
	
    // same points in homogeneous coordinates
    Mat v1(3, 1, CV_32FC1);
    v1.at<float>(0, 0) = x.at<float>(0, 0);
    v1.at<float>(1, 0) = x.at<float>(1, 0);
    v1.at<float>(2, 0) = 1;
    // define v2 as homogeneous version of y
	Mat v2(3, 1, CV_32FC1);
	v2.at<float>(0, 0) = y.at<float>(0, 0);
	v2.at<float>(1, 0) = y.at<float>(1, 0);
	v2.at<float>(2, 0) = 1;
    
    // print points
    cout << "point 1: " << v1.t() << "^T" << endl;
    cout << "point 2: " << v2.t() << "^T" << endl;
    cout << endl;
    
    // the connecting line between those points in homogeneous coordinates
    Mat line = getConnectingLine(v1, v2);
    
    // print line
    cout << "joining line: " << line << "^T" << endl;
    cout << endl;    
    
    // the parameters of the transformation
    int dx = 6;				// translation in x
    int dy = -7;			// translation in y
    double phi = 15;		// rotation angle in degree
    double lambda = 8;		// scaling factor

    // matrices for transformation
    // calculate translation matrix
    Mat T = getTranslMatrix(dx, dy);
    // calculate rotation matrix
    Mat R = getRotMatrix(phi);
    // calculate scale matrix
    Mat S = getScaleMatrix(lambda);
    // combine individual transformations to a homography
    Mat H = getH(T, R, S);
    
    // print calculated matrices
    cout << "Translation matrix: " << endl;
    cout << T << endl;
    cout << endl;
    cout << "Rotation matrix: " << endl;
    cout << R << endl;
    cout << endl;
    cout << "Scaling matrix: " << endl;
    cout << S << endl;
    cout << endl;
    cout << "Homography: " << endl;
    cout << H << endl;
    cout << endl;

    // transform first point x (and print it)
    Mat v1_new = applyH(v1, H, "point");
    cout << "new point 1: " << v1_new << "^T" << endl;
    // transform second point y (and print it)
    Mat v2_new = applyH(v2, H, "point");
    cout << "new point 2: " << v2_new << "^T" << endl;
    cout << endl;
    // transform joining line (and print it)
    Mat line_new = applyH(line, H, "line");
    cout << "new line: " << line_new << "^T" << endl;
    cout << endl;

    // check if transformed points are still on transformed line
    bool xOnLine = isPointOnLine(v1_new, line_new);
    bool yOnLine = isPointOnLine(v2_new, line_new);
    if (xOnLine)
		cout << "first point lies still on the line *yay*" << endl;
    else
		cout << "first point does not lie on the line *oh oh*" << endl;

    if (yOnLine)
		cout << "second point lies still on the line *yay*" << endl;
    else
		cout << "second point does not lie on the line *oh oh*" << endl;

}

// function loads input image and calls processing function
// output is tested on "correctness" 
/*
fname	path to input image
*/
void Pcv1::test(string fname){

	// some image variables
	Mat inputImage, outputImage;
  
	// load image
	inputImage = imread( fname );

	// check if image can be loaded
	if (!inputImage.data){
	    cout << "ERROR: Cannot read file " << fname << endl;
	    cout << "Press enter to continue..." << endl;
	    cin.get();
	    exit(-1);
	}
	
	test_getConnectingLine();
	test_getScaleMatrix();
	test_getRotMatrix();
	test_getTranslMatrix();
	test_getH();
	test_applyH();
	test_isPointOnLine();
	
	cout << "Finished basic testing: Everything seems to be fine." << endl;
}

void Pcv1::test_getConnectingLine(void){

	Mat v1 = (Mat_<float>(3,1) << 0, 0, 1);
	Mat v2 = (Mat_<float>(3,1) << 1, 1, 1);
	Mat lt = (Mat_<float>(3,1) << -1, 1, 0);
	Mat lc = getConnectingLine(v1, v2);
	
	if (sum(lc != lt).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::getConnectingLine(..)!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}	
}

void Pcv1::test_getScaleMatrix(void){

	Mat St = (Mat_<float>(3,3) << 3, 0, 0, 0, 3, 0, 0, 0, 1);
	Mat Sc = getScaleMatrix(3);
	
	if (sum(Sc != St).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::getScaleMatrix(..)!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}	
}

void Pcv1::test_getRotMatrix(void){

	Mat Rt = (Mat_<float>(3,3) << 1./sqrt(2), -1./sqrt(2), 0, 1./sqrt(2), 1./sqrt(2), 0, 0, 0, 1);
	Mat Rc = getRotMatrix(45);
	
	if (sum(Rc != Rt).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::getRotMatrix(..)!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}	
}

void Pcv1::test_getTranslMatrix(void){

	Mat Tt = (Mat_<float>(3,3) << 1, 0, -1, 0, 1, -1, 0, 0, 1);
	Mat Tc = getTranslMatrix(-1,-1);
	
	if (sum(Tc != Tt).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::getTranslMatrix(..)!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}	
}

void Pcv1::test_getH(void){

	Mat St = (Mat_<float>(3,3) << 3, 0, 0, 0, 3, 0, 0, 0, 1);
	Mat Rt = (Mat_<float>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
	Mat Tt = (Mat_<float>(3,3) << 1, 0, -1, 0, 1, -1, 0, 0, 1);
	Mat Ht = (Mat_<float>(3,3) << 0, -3, 3, 3, 0, -3, 0, 0, 1);
	Mat Hc = getH(Tt, Rt, St);

	if (sum(Hc != Ht).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::getH(..)!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}
}
void Pcv1::test_applyH(void){

	Mat H = (Mat_<float>(3,3) << 0, -3, 3, 3, 0, -3, 0, 0, 1);
	Mat v = (Mat_<float>(3,1) << 1, 1, 1);
	Mat vnt = (Mat_<float>(3,1) << 0, 0, 1);
	Mat l = (Mat_<float>(3,1) << -1, 1, 0);
	Mat lnt = (Mat_<float>(3,1) << -1, -1, 0)/3.;
	
	Mat vnc = applyH(v, H, "point");
	Mat lnc = applyH(l, H, "line");

	if (sum(vnc != vnt).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::applyH(..) for points!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}
	if (sum(lnc != lnt).val[0] != 0){
		cout << "There seems to be a problem with Pcv1::applyH(..) for lines!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}
}

void Pcv1::test_isPointOnLine(void){
	
	Mat v = (Mat_<float>(3,1) << 1, 1, 1);
	Mat l = (Mat_<float>(3,1) << -1, 1, 0);

	if (!isPointOnLine(v, l)){
		cout << "There seems to be a problem with Pcv1::isPointOnLine(..)!" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}
}