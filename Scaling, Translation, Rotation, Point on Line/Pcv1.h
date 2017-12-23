//============================================================================
// Name        : Pcv1.h
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : header file for first PCV assignment
//============================================================================

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


class Pcv1{
	
	public:
		// constructor
		Pcv1(void){};
		// destructor
		~Pcv1(void){};
		
		// processing routine
		void run(string);
		// testing routine
		void test(string);

	private:
		// --> edit ONLY these functions!
		bool isPointOnLine(Mat& point, Mat& line, double eps = pow(10,-5));
		Mat applyH(Mat& geomObj, Mat& H, string type);
		Mat getH(Mat& T, Mat& R, Mat& S);
		Mat getScaleMatrix(double lambda);
		Mat getRotMatrix(double phi);
		Mat getTranslMatrix(double dx, double dy);
		Mat getConnectingLine(Mat& p1, Mat& p2);

		// test functions
		void test_isPointOnLine(void);
		void test_applyH(void);
		void test_getH(void);
		void test_getScaleMatrix(void);
		void test_getRotMatrix(void);
		void test_getTranslMatrix(void);
		void test_getConnectingLine(void);
};
