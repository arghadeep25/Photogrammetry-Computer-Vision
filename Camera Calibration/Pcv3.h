//============================================================================
// Name        : Pcv3.h
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : header file for the third PCV assignment
//============================================================================

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct winInfo {Mat img; string name; vector<Point2f> pointList;};

class Pcv3{

	public:
		// constructor
		Pcv3(void){};
		// destructor
		~Pcv3(void){};
		
		// processing routine
		void run(string, string);
		// testing routine
		void test(void);

	protected:
		// given functions
		int getPoints(struct winInfo& calib, string fname, Mat& points2D, Mat& points3D);
		// functions to be implemented
		// --> please edit ONLY these functions!
		Mat calibrate(Mat&, Mat&);
		Mat solve_dlt(Mat&);
		void decondition(Mat&, Mat&, Mat&);
		Mat getDesignMatrix_camera(Mat& points2D, Mat& points3D);
		Mat applyH(Mat&, Mat&, string);
		Mat getCondition2D(Mat&);
		Mat getCondition3D(Mat&);
		void interprete(Mat&, vector<Mat>&);

		// test functions
		bool test_getCondition2D(void);
		bool test_getCondition3D(void);
		bool test_getDesignMatrix_camera(void);
		bool test_solve_dlt(void);
		bool test_decondition(void);
		bool test_calibrate(void);
		bool test_interprete(void);
};
