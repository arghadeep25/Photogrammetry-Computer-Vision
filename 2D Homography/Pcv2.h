//============================================================================
// Name        : Pcv2.h
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : header file for second PCV assignment
//============================================================================

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct winInfo {Mat img; string name; vector<Point2f> pointList;};

class Pcv2{

	public:
		// constructor
		Pcv2(void){};
		// destructor
		~Pcv2(void){};
		
		// processing routine
		void run(string, string, string);
		// testing routine
		void test(void);

	private:
		// given functions
		int getPoints(struct winInfo&, struct winInfo&, Mat&, Mat&);
		Mat stitch(Mat&, Mat&, Mat&);
		// functions to be implemented
		// --> please edit ONLY these functions!
		Mat homography2D(Mat&, Mat&);
		void decondition(Mat&, Mat&, Mat&);
		Mat getDesignMatrix_homography2D(Mat&, Mat&);
		Mat applyH(Mat&, Mat&, string);
		Mat getCondition2D(Mat&);
		Mat solve_dlt(Mat&);

		// testing functions
		void test_homography2D(void);
		void test_decondition(void);
		void test_getDesignMatrix_homography2D(void);
		void test_getCondition2D(void);
		void test_solve_dlt(void);
		
};
