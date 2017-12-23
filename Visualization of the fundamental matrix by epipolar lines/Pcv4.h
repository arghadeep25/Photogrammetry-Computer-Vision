//============================================================================
// Name        : Pcv4.h
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : header file for fourth PCV assignment
//============================================================================

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct winInfo {Mat img; string name; vector<Point2f> pointList;};

class Pcv4{

	public:
		// constructor
		Pcv4(void){};
		// destructor
		~Pcv4(void){};
		
		// processing routine
		void run(string, string);

	private:
		// given functions
		int getPoints(struct winInfo& img1, struct winInfo& img2, Mat& p1, Mat& p2);
		void drawEpiLine(Mat& img, double a, double b, double c);
		// functions to be implemented
		// --> please edit ONLY these functions!
		Mat getFundamentalMatrix(Mat& p1, Mat& p2);
		Mat getCondition2D(Mat& p);
		Mat applyH(Mat& geomObj, Mat& H, string type);
		Mat getDesignMatrix_fundamental(Mat& n1, Mat& n2);
		void forceSingularity(Mat& F);
		Mat solve_dlt(Mat& A);
		void decondition(Mat& T1, Mat& T2, Mat& F);
		void visualize(struct winInfo& img1, struct winInfo& img2, Mat& p_fst, Mat& p_snd, Mat& F);
		double getError(Mat& p_fst, Mat& p_snd, Mat& F);

};
