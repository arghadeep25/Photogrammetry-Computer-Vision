//============================================================================
// Name        : Pcv3.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : Camera calibration
//============================================================================

// Dear Ronny, the code doesn't work and still need one part to do

#include "Pcv3.h"

// extract and prints information about interior and exterior orientation from camera
/*
P:	The 3x4 projection matrix
info:	1st entry: K
2nd entry: R
3rd entry: Vector of all 11 parameters (order see slides)
*/
void Pcv3::interprete(Mat& P, vector<Mat>& info) {
	Mat M = P(cv::Rect(0, 0, 3, 3));
	Mat R, Q, Qx, Qy, Qz, Rot, Cal;

	double lambda = 1 / sqrt(M.row(2).dot(M.row(2)));
	if (cv::determinant(M) <= 0)
		lambda = -lambda;
		P = lambda*P;
		cv::RQDecomp3x3(M, R, Q, Qx, Qy, Qz);
		Rot = Q;
		Cal = R;

	for (int i = 0; i < 3; i++) {
		if (R.at<float>(i, i) < 0) {
			Q.row(i) = -1 * Q.row(i);
		}
	}
	Mat par = Mat(1, 11, CV_32FC1);
	par.at<float>(0, 0) = Cal.at<float>(0, 0);
	par.at<float>(0, 1) = Cal.at<float>(0, 1);
	par.at<float>(0, 1) = atan(-Cal.at<float>(0, 0) / Cal.at<float>(1, 1)) * 180 / 3.1415926535;
	par.at<float>(0, 2) = Cal.at<float>(1, 1) / Cal.at<float>(0, 0);
	par.at<float>(0, 3) = Cal.at<float>(0, 2);
	par.at<float>(0, 4) = Cal.at<float>(0, 1);
	par.at<float>(0, 5) = acos(Qx.at<float>(1, 1)) * 180 / 3.1415926535;//omega
	par.at<float>(0, 6) = -acos(Qy.at<float>(0, 0)) * 180 / 3.1415926535;//phi
	par.at<float>(0, 7) = acos(Qz.at<float>(0, 0)) * 180 / 3.1415926535;//kapa

	cv::SVD svd(P, SVD::FULL_UV);
	Mat temp = -M.inv()*P.col(3);
	par.at<float>(0, 8) = temp.at<float>(0);
	par.at<float>(0, 9) = temp.at<float>(1);
	par.at<float>(0, 10) = temp.at<float>(2);
}

// estimate projection matrix
/*
points2D	set of 2D points within the image
points3D	set of 3D points at the object
return		the projection matrix to be computed
*/
Mat Pcv3::calibrate(Mat& points2D, Mat& points3D) {
	// TO DO !!!
	Mat T_2D = getCondition2D(points2D);
	Mat T_3D = getCondition3D(points3D);
	Mat points2D_cond = applyH(points2D, T_2D, "point");
	Mat points3D_cond = applyH(points3D, T_3D, "point");
	Mat A = getDesignMatrix_camera(points2D_cond, points3D_cond);
	Mat P = solve_dlt(A);
	decondition(T_2D, T_3D, P);
	return P;
}

// solve homogeneous equation system by usage of SVD
/*
A			the design matrix
return		the estimated projection matrix
*/
Mat Pcv3::solve_dlt(Mat& A) {
	// TO DO !!!
	Mat p = Mat(3, 4, CV_32FC1);
	cv::SVD svd(A, SVD::FULL_UV);
	p = svd.vt.row(A.rows - 1).reshape(0, 3);
	return p;
}

// decondition a projection matrix that was estimated from conditioned point clouds
/*
T_2D	conditioning matrix of set of 2D image points
T_3D	conditioning matrix of set of 3D object points
P	conditioned projection matrix that has to be un-conditioned (in-place)
*/
void Pcv3::decondition(Mat& T_2D, Mat& T_3D, Mat& P) {
	// TO DO !!!
	P = T_2D.inv() * P * T_3D;
}

// define the design matrix as needed to compute projection matrix
/*
points2D	set of 2D points within the image
points3D	set of 3D points at the object
return		the design matrix to be computed
*/
Mat Pcv3::getDesignMatrix_camera(Mat& points2D, Mat& points3D) {
	// TO DO !!!
	int n = points2D.cols;//number of points
	Mat H = Mat::zeros(2 * n, 12, CV_32FC1);
	double u, v, w, U, V, W, Q;
	Mat temp;
	for (int i = 0; i < n; i++) {
		u = points2D.at<float>(0, i);
		v = points2D.at<float>(1, i);
		w = points2D.at<float>(2, i);
		U = points3D.at<float>(0, i);
		V = points3D.at<float>(1, i);
		W = points3D.at<float>(2, i);
		Q = points3D.at<float>(3, i);
		temp = (Mat_<float>(1, 12) << -w*U, -w*V, w*W, w*Q, 0, 0, 0, 0, u*U, u*V, u*W, u*Q);
		temp.copyTo(H.row(2 * i));
		temp = (Mat_<float>(1, 12) << 0, 0, 0, 0, -w*U, -w*V, -w*W, -w*Q, v*U, v*V, v*W, v*Q);
		temp.copyTo(H.row(2 * i + 1));
	}
	return H;
}

// apply transformation to set of points
/*
H			matrix representing the transformation
geomObj		matrix with input objects (one per column)
type		the type of the geometric object (for now: only point and line)
return		transformed objects (one per column)
*/
Mat Pcv3::applyH(Mat& geomObj, Mat& H, string type) {
	// TO DO !!!
	// if point
	if (type.compare("point") == 0) {
		return H*geomObj;
	}
	// if line
	if (type.compare("line") == 0) {
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
Mat Pcv3::getCondition2D(Mat& p) {
	// TO DO !!!
	Mat one = Mat::ones(1, p.cols, CV_32FC1);
	double tx = p.row(0).dot(one) / p.cols;
	double ty = p.row(1).dot(one) / p.cols;
	double sx = (abs(tx*one - p.row(0))).dot(one) / p.cols;
	double sy = (abs(ty*one - p.row(1))).dot(one) / p.cols;
	Mat T = Mat::eye(3, 3, CV_32S);
	if (sx != 0 && sy != 0) {
		T = (Mat_<float>(3, 3) << 1 / sx, 0, -tx / sx, 0, 1 / sy, -ty / sy, 0, 0, 1);
	}
	return T;
}

// get the conditioning matrix of given points
/*
p		the points as matrix
return		the condition matrix
*/
Mat Pcv3::getCondition3D(Mat& p) {
	// TO DO !!!
	Mat one = Mat::ones(1, p.cols, CV_32FC1);
	double tx = p.row(0).dot(one) / p.cols;
	double ty = p.row(1).dot(one) / p.cols;
	double tz = p.row(1).dot(one) / p.cols;
	double sx = (abs(tx*one - p.row(0))).dot(one) / p.cols;
	double sy = (abs(ty*one - p.row(1))).dot(one) / p.cols;
	double sz = (abs(tz*one - p.row(2))).dot(one) / p.cols;
	Mat T = Mat::eye(4, 4, CV_32S);
	if (sx != 0 && sy != 0 && sz != 0) {
		T = (Mat_<float>(4, 4) << 1 / sx, 0, 0, -tx / sx, 0, 1 / sy, 0, -ty / sy, 0, 0, 1 / sz, -tz / sz, 0, 0, 0, 1);
	}
	return T;
}

/* *****************************
GIVEN FUNCTIONS
***************************** */

// function loads input image, calls processing functions, and saves the result
/*
fname	path to input image
*/
void Pcv3::run(string imgPath, string pointPath) {

	// titles of point-selection windows
	string winName1 = string("Calibration image");

	// load image calibration image, paths in argv[1]
	Mat calibImage = imread(imgPath);
	if (!calibImage.data) {
		cerr << "ERROR: Could not load image " << imgPath << endl;
		cerr << "Press enter to continue..." << endl;
		cin.get();
		exit(-1);
	}
	// fuse image data and window title
	struct winInfo calib;
	calib.img = calibImage.clone();
	calib.name = winName1;

	// get corresponding points within the image
	Mat points2D, points3D;
	string fname = pointPath;
	int numberOfPointPairs = getPoints(calib, fname, points2D, points3D);

	// just some putput
	cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
	cout << endl << "2D Points in image:" << endl;
	cout << points2D << endl;
	cout << endl << "3D Points at object:" << endl;
	cout << points3D << endl;

	// calculate the projection matrix
	Mat P = calibrate(points2D, points3D);

	// decompose P to get camera parameter
	vector<Mat> params;
	interprete(P, params);

	cout << endl << "Calibration matrix: " << endl;
	cout << params.at(0) << endl;
	cout << endl << "Rotation matrix: " << endl;
	cout << params.at(1) << endl;

	cout << endl << "Principle distance:\t\t" << params.at(2).at<float>(0) << endl
		<< "Skew:\t\t\t\t" << params.at(2).at<float>(1) << endl
		<< "Aspect ratio:\t\t\t" << params.at(2).at<float>(2) << endl
		<< "Principle point (x,y):\t\t[" << params.at(2).at<float>(3) << ", " << params.at(2).at<float>(4) << "]^T" << endl;

	cout << endl << "omega:\t" << params.at(2).at<float>(5) << endl
		<< "phi:\t" << params.at(2).at<float>(6) << endl
		<< "kappa:\t" << params.at(2).at<float>(7) << endl;

	cout << endl << "external position:\t[" << params.at(2).at<float>(8) << ", " << params.at(2).at<float>(9) << ", " << params.at(2).at<float>(10) << "]^T" << endl;

}

// mouse call back to get points and draw circles
/*
event	specifies encountered mouse event
x,y	position of mouse pointer
flags	not used here
param	a struct containing used IplImage and window title
*/
void getPointsCB(int event, int x, int y, int flags, void* param) {

	// cast to structure
	struct winInfo* win = (struct winInfo*)param;

	switch (event) {
		// if left mouse button was pressed
	case CV_EVENT_LBUTTONDOWN: {
		// create point representing mouse position
		Point2f p = Point2f(x, y);
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

// display image and catch points marked by left mouse clicks
// points have to be clicked in same order as in the file
// points will be in homogeneous coordinates
/*
calib		structure containing calibration image
fname		path to file that contains 3D real world (image points have to be defined in same order)
points2D	points within the image (to be defined by this method)
points3D	points at the object (to be read from file)
*/
int Pcv3::getPoints(struct winInfo& calib, string fname, Mat& points2D, Mat& points3D) {

	// show input images and install mouse callback
	namedWindow(calib.name.c_str(), 0);
	imshow(calib.name.c_str(), calib.img);
	setMouseCallback(calib.name.c_str(), getPointsCB, (void*)(&calib));
	// wait until any key was pressed
	waitKey(0);

	/* Tired of clicking the same points all over again?
	* You can specify them here as well
	* eg. like this:*/
	/*calib.pointList.clear();
	calib.pointList.push_back(Point2f(18.5,   46.8));
	calib.pointList.push_back(Point2f(99.1,  146.5));
	calib.pointList.push_back(Point2f(13.8,  221.8));
	calib.pointList.push_back(Point2f(242.1,  52.5));
	calib.pointList.push_back(Point2f(151.1, 147.1));
	calib.pointList.push_back(Point2f(243.1, 224.5));*/

	// allocate memory for point-lists (represented as matrix)
	points2D = Mat(3, calib.pointList.size(), CV_32FC1);
	points3D = Mat(4, calib.pointList.size(), CV_32FC1);
	int n = 0;	// number of point pairs

	fstream calibFile(fname.c_str(), ios::in);

	string buffer;
	int e;
	// read points from global variable, transform them into homogeneous coordinates
	for (vector<Point2f>::iterator p = calib.pointList.begin(); p != calib.pointList.end(); p++, n++) {
		// define image point
		points2D.at<float>(0, n) = p->x;
		points2D.at<float>(1, n) = p->y;
		points2D.at<float>(2, n) = 1;

		// read object points from file 
		getline(calibFile, buffer);
		e = buffer.find(" ");
		points3D.at<float>(0, n) = atof(buffer.substr(0, e).c_str());
		buffer = buffer.substr(e + 1, string::npos);
		e = buffer.find(" ");
		points3D.at<float>(1, n) = atof(buffer.substr(0, e).c_str());
		buffer = buffer.substr(e + 1, string::npos);
		points3D.at<float>(2, n) = atof(buffer.c_str());
		points3D.at<float>(3, n) = 1;
	}

	// be tidy
	destroyWindow(calib.name.c_str());
	calibFile.close();

	return n;

}

// function calls processing functions
// output is tested on "correctness" 
void Pcv3::test(void) {
	bool correct = true;
	cout << endl << "********************" << endl;
	cout << "Testing: Start" << endl;
	correct = test_getCondition2D();
	correct = test_getCondition3D();
	correct = test_getDesignMatrix_camera();
	correct = test_solve_dlt();
	correct = test_decondition();
	correct = test_calibrate();
	correct = test_interprete();
	cout << "Testing: Done" << endl;
	if (correct)
		cout << "Everything seems (!) to be correct." << endl;
	else
		cout << "There seem to be problems." << endl;
	cout << endl << "********************" << endl << endl;
}

bool Pcv3::test_getCondition2D(void) {

	bool correct = true;

	Mat p = (Mat_<float>(3, 6) << 18.5, 99.1, 13.8, 242.1,
		151.1, 243.1, 46.8, 146.5,
		221.8, 52.5, 147.1, 224.5,
		1, 1, 1, 1, 1, 1);
	Mat Ttrue = (Mat_<float>(3, 3) << 0.011883541, 0, -1.5204991, 0, 0.016626639, -2.3255126, 0, 0, 1);

	Mat Test = getCondition2D(p);
	if ((Test.rows != 3) || (Test.cols != 3) || (Test.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::getCondition2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cout << "\t==> Expected 3x3, but got " << Test.rows << "x" << Test.cols << "." << endl;
		correct = false;
		cin.get();
	}
	Test.convertTo(Test, CV_32FC1);
	Test = Test / Test.at<float>(2, 2);
	double eps = pow(10, -3);
	if (sum(abs(Test - Ttrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::getCondition2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cout << "\t==> Expected:" << Ttrue << endl;
		cout << "\t==> But got:" << Test << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

bool Pcv3::test_getCondition3D(void) {

	bool correct = true;

	Mat p = (Mat_<float>(4, 6) << 44.7, -103.6, 47.4, -152.2,
		-153.3, -149.4, -142.4, -146.6,
		-150.1, 59.4, -96.9, 52.7, 258.7,
		154.4, 59.8, 245.2, 151.3, 46.9,
		1, 1, 1, 1, 1, 1);
	Mat Ttrue = (Mat_<float>(4, 4) << 0.012117948, 0, 0, 0.9419685, 0, 0.011838989, 0, 0.83642465, 0, 0, 0.014988759, -2.2890332, 0, 0, 0, 1);

	Mat Test = getCondition3D(p);
	if ((Test.rows != 4) || (Test.cols != 4) || (Test.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::getCondition3D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cout << "\t==> Expected 4x4, but got " << Test.rows << "x" << Test.cols << "." << endl;
		correct = false;
		cin.get();
	}
	Test.convertTo(Test, CV_32FC1);
	Test = Test / Test.at<float>(3, 3);
	double eps = pow(10, -3);
	if (sum(abs(Test - Ttrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::getCondition3D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cout << "\t==> Expected:" << Ttrue << endl;
		cout << "\t==> But got:" << Test << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

bool Pcv3::test_getDesignMatrix_camera(void) {

	bool correct = true;

	Mat p1 = (Mat_<float>(3, 6) << -1.3006536, -0.34284019, -1.3565062, 1.3565062, 0.27510405, 1.3683897, -1.5473859,
		0.11029005, 1.3622761, -1.4526141, 0.1202662, 1.4071679, 1, 1, 1, 1, 1, 1);
	Mat p2 = (Mat_<float>(4, 6) << 1.4836408, -0.31345099, 1.5163593, -0.90238315, -0.91571301, -0.86845297, -0.84944731,
		-0.89917129, -0.94060773, 1.5396607, -0.31077343, 1.4603393, 1.5885589, 0.025231123,
		-1.3927054, 1.3862104, -0.021234035, -1.5860603, 1, 1, 1, 1, 1, 1);

	Mat Aest = getDesignMatrix_camera(p1, p2);
	if ((Aest.rows != 12) || (Aest.cols != 12) || (Aest.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::getDesignMatrix_camera(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cout << "\t==> Expected 12x12, but got " << Aest.rows << "x" << Aest.cols << "." << endl;
		correct = false;
		cin.get();
	}
	Aest.convertTo(Aest, CV_32FC1);
	Mat Atrue = (Mat_<float>(12, 12) << -1.4836408, 0.84944731, -1.5885589, -1, 0, 0, 0, 0, -1.9297028, 1.1048367, -2.0661647,
		-1.3006536, 0, 0, 0, 0, -1.4836408, 0.84944731, -1.5885589, -1, -2.2957649, 1.3144228,
		-2.4581137, -1.5473859, 0.31345099, 0.89917129, -0.025231123, -1, 0, 0, 0, 0, 0.1074636,
		0.30827206, -0.0086502433, -0.34284019, 0, 0, 0, 0, 0.31345099, 0.89917129, -0.025231123,
		-1, -0.034570526, -0.099169649, 0.0027827418, 0.11029005, -1.5163593, 0.94060773,
		1.3927054, -1, 0, 0, 0, 0, -2.0569508, 1.2759403, 1.8892136, -1.3565062, 0, 0, 0, 0,
		-1.5163593, 0.94060773, 1.3927054, -1, 2.0657001, -1.2813674, -1.8972493, 1.3622761, 0.90238315,
		-1.5396607, -1.3862104, -1, 0, 0, 0, 0, -1.2240883, 2.0885594, 1.880403, 1.3565062, 0, 0, 0, 0, 0.90238315,
		-1.5396607, -1.3862104, -1, 1.3108145, -2.2365327, -2.0136287, -1.4526141, 0.91571301, 0.31077343, 0.021234035,
		-1, 0, 0, 0, 0, -0.25191635, -0.085495025, -0.005841569, 0.27510405, 0, 0, 0, 0, 0.91571301,
		0.31077343, 0.021234035, -1, -0.11012933, -0.03737554, -0.0025537368, 0.1202662, 0.86845297,
		-1.4603393, 1.5860603, -1, 0, 0, 0, 0, -1.1883821, 1.9983133, -2.1703486, 1.3683897, 0, 0, 0, 0,
		0.86845297, -1.4603393, 1.5860603, -1, -1.2220591, 2.0549426, -2.2318532, 1.4071679);

	double eps = pow(10, -3);
	if (sum(abs(Aest - Atrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::getDesignMatrix_camera(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cout << "\t==> Expected:" << Atrue << endl;
		cout << "\t==> But got:" << Aest << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

bool Pcv3::test_solve_dlt(void) {

	bool correct = true;

	Mat A = (Mat_<float>(12, 12) << -1.4836408, 0.84944731, -1.5885589, -1, 0, 0, 0, 0, -1.9297028, 1.1048367, -2.0661647, -1.3006536,
		0, 0, 0, 0, -1.4836408, 0.84944731, -1.5885589, -1, -2.2957649, 1.3144228, -2.4581137, -1.5473859,
		0.31345099, 0.89917129, -0.025231123, -1, 0, 0, 0, 0, 0.1074636, 0.30827206, -0.0086502433, -0.34284019,
		0, 0, 0, 0, 0.31345099, 0.89917129, -0.025231123, -1, -0.034570526, -0.099169649, 0.0027827418, 0.11029005,
		-1.5163593, 0.94060773, 1.3927054, -1, 0, 0, 0, 0, -2.0569508, 1.2759403, 1.8892136, -1.3565062, 0, 0, 0, 0,
		-1.5163593, 0.94060773, 1.3927054, -1, 2.0657001, -1.2813674, -1.8972493, 1.3622761, 0.90238315, -1.5396607,
		-1.3862104, -1, 0, 0, 0, 0, -1.2240883, 2.0885594, 1.880403, 1.3565062, 0, 0, 0, 0, 0.90238315, -1.5396607,
		-1.3862104, -1, 1.3108145, -2.2365327, -2.0136287, -1.4526141, 0.91571301, 0.31077343, 0.021234035,
		-1, 0, 0, 0, 0, -0.25191635, -0.085495025, -0.005841569, 0.27510405, 0, 0, 0, 0, 0.91571301, 0.31077343,
		0.021234035, -1, -0.11012933, -0.03737554, -0.0025537368, 0.1202662, 0.86845297, -1.4603393, 1.5860603,
		-1, 0, 0, 0, 0, -1.1883821, 1.9983133, -2.1703486, 1.3683897, 0, 0, 0, 0, 0.86845297, -1.4603393, 1.5860603,
		-1, -1.2220591, 2.0549426, -2.2318532, 1.4071679);
	Mat Pest = solve_dlt(A);
	if ((Pest.rows != 3) || (Pest.cols != 4) || (Pest.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::solve_dlt(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cout << "\t==> Expected 3x4, but got " << Pest.rows << "x" << Pest.cols << "." << endl;
		correct = false;
		cin.get();
	}
	Pest.convertTo(Pest, CV_32FC1);
	Pest = Pest / Pest.at<float>(2, 3);
	Mat Ptrue = (Mat_<float>(3, 4) << -0.32120419, 0.3686696, -0.0095243761, 0.0035335352, -0.052101973, -0.083373591, -0.59302819, -0.0046088211, -0.030212782, -0.026015088, 0.0050823218, 0.63073134);
	Ptrue = Ptrue / Ptrue.at<float>(2, 3);

	double eps = pow(10, -3);
	if (sum(abs(Pest - Ptrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::solve_dlt(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cout << "\t==> Expected:" << Ptrue << endl;
		cout << "\t==> But got:" << Pest << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

bool Pcv3::test_decondition(void) {

	bool correct = true;

	Mat P = (Mat_<float>(3, 4) << -0.32120419, 0.3686696, -0.0095243761, 0.0035335352, -0.052101973, -0.083373591, -0.59302819, -0.0046088211, -0.030212782, -0.026015088, 0.0050823218, 0.63073134);
	Mat T1 = (Mat_<float>(3, 3) << 0.011883541, 0, -1.5204991, 0, 0.016626639, -2.3255126, 0, 0, 1);
	Mat T2 = (Mat_<float>(4, 4) << 0.012117948, 0, 0, 0.9419685, 0, 0.011838989, 0, 0.83642465, 0, 0, 0.014988759, -2.2890332, 0, 0, 0, 1);
	decondition(T1, T2, P);
	if ((P.rows != 3) || (P.cols != 4) || (P.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::decondition(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cout << "\t==> Expected 3x4, but got " << P.rows << "x" << P.cols << "." << endl;
		correct = false;
		cin.get();
	}
	P.convertTo(P, CV_32FC1);
	P = P / P.at<float>(2, 3);
	Mat Ptrue = (Mat_<float>(3, 4) << -0.65811008, 0.57636172, -0.0039836233, 132.55562, -0.15676615, -0.18008056, -0.9210307, 270.33484, -0.00064357655, -0.00054140261, 0.00013390853, 1);
	double eps = pow(10, -3);
	if (sum(abs(P - Ptrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::decondition(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cout << "\t==> Expected:" << Ptrue << endl;
		cout << "\t==> But got:" << P << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

bool Pcv3::test_calibrate(void) {

	bool correct = true;

	Mat p1 = (Mat_<float>(3, 6) << 18.5, 99.1, 13.8, 242.1, 151.1, 243.1, 46.8, 146.5, 221.8, 52.5, 147.1, 224.5, 1, 1, 1, 1, 1, 1);
	Mat p2 = (Mat_<float>(4, 6) << 44.7, -103.6, 47.4, -152.2, -153.3, -149.4, -142.4, -146.6, -150.1, 59.4, -96.9, 52.7, 258.7, 154.4, 59.8, 245.2, 151.3, 46.9, 1, 1, 1, 1, 1, 1);

	Mat Pest = calibrate(p1, p2);
	if ((Pest.rows != 3) || (Pest.cols != 4) || (Pest.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::calibrate(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cout << "\t==> Expected 3x4, but got " << Pest.rows << "x" << Pest.cols << "." << endl;
		correct = false;
		cin.get();
	}
	Pest.convertTo(Pest, CV_32FC1);
	Pest = Pest / Pest.at<float>(2, 3);
	Mat Ptrue = (Mat_<float>(3, 4) << -0.65811008, 0.57636172, -0.0039836233, 132.55562, -0.15676615, -0.18008056, -0.9210307, 270.33484, -0.00064357655, -0.00054140261, 0.00013390853, 1);
	double eps = pow(10, -3);
	if (sum(abs(Pest - Ptrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::calibrate(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cout << "\t==> Expected:" << Ptrue << endl;
		cout << "\t==> But got:" << Pest << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

bool Pcv3::test_interprete(void) {

	bool correct = true;

	Mat P = (Mat_<float>(3, 4) << -0.65811008, 0.57636172, -0.0039836233, 132.55562, -0.15676615, -0.18008056, -0.9210307, 270.33484, -0.00064357655, -0.00054140261, 0.00013390853, 1);

	vector<Mat> info;
	interprete(P, info);

	if (info.size() != 3) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong number of matrices (expected 3, but got " << info.size() << ")!" << endl;
		correct = false;
		cin.get();
	}

	Mat Kest = info.at(0);
	Mat Rest = info.at(1);
	Mat paramsest = info.at(2);

	if ((Kest.rows != 3) || (Kest.cols != 3) || (Kest.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong dimensions of K!" << endl;
		cout << "\t==> Expected 3x3, but got " << Kest.rows << "x" << Kest.cols << "." << endl;
		correct = false;
		cin.get();
	}
	if ((Rest.rows != 3) || (Rest.cols != 3) || (Rest.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong dimensions of R!" << endl;
		cout << "\t==> Expected 3x3, but got " << Rest.rows << "x" << Rest.cols << "." << endl;
		correct = false;
		cin.get();
	}
	if (((paramsest.rows != 11) && (paramsest.cols != 11)) || (paramsest.channels() != 1)) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong number of parameters!" << endl;
		cout << "\t==> Expected 11, but got " << max(paramsest.rows, paramsest.cols) << "." << endl;
		correct = false;
		cin.get();
	}
	Kest.convertTo(Kest, CV_32FC1);
	Kest = Kest / Kest.at<float>(2, 2);
	Mat Ktrue = (Mat_<float>(3, 3) << 1015.7469, -10.457143, 153.00758, 0, 1112.4618, 103.48759, 0, 0, 1);
	double eps = pow(10, -3);
	if (sum(abs(Kest - Ktrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations for K!" << endl;
		cout << "\t==> Expected:" << Ktrue << endl;
		cout << "\t==> But got:" << Kest << endl;
		correct = false;
		cin.get();
	}
	Rest.convertTo(Rest, CV_32FC1);
	Mat Rtrue = (Mat_<float>(3, 3) << -0.64794523, 0.76071578, -0.038450673, -0.095171571, -0.13094184, -0.98681134, -0.75571775, -0.63574028, 0.15724166);
	if (sum(abs(Rest - Rtrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations for R!" << endl;
		cout << "\t==> Expected:" << Rtrue << endl;
		cout << "\t==> But got:" << Rest << endl;
		correct = false;
		cin.get();
	}
	paramsest.convertTo(paramsest, CV_32FC1);
	Mat paramstrue = (Mat_<float>(1, 11) << 1015.7469, 89.410156, 1.0952156, 153.00758, 103.48759, 76.107491, -49.088123, 171.64403, 890.0155, 786.18341, -11.688845);
	if (sum(abs(paramsest - paramstrue)).val[0] > eps) {
		cout << "Warning: There seems to be a problem with Pcv3::interprete(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations for parameters!" << endl;
		cout << "\t==> Expected:" << paramstrue << endl;
		cout << "\t==> But got:" << paramsest << endl;
		correct = false;
		cin.get();
	}
	return correct;
}

