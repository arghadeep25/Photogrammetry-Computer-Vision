//============================================================================
// Name        : Pcv4.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : Estimation of Fundamental Matrix
//============================================================================

#include "Pcv4.h"

// compute the fundamental matrix
/*
fst	first set of points
snd	second set of points
return	the estimated fundamental matrix
*/
Mat Pcv4::getFundamentalMatrix(Mat& fst, Mat& snd){
	// TO DO !!!
	Mat T_fst = Pcv4::getCondition2D(fst);
	Mat T_snd = Pcv4::getCondition2D(snd);
	Mat fst_con = applyH(fst, T_fst, "point");
	Mat snd_con = applyH(snd, T_snd, "point");
	Mat A = Pcv4::getDesignMatrix_fundamental(fst_con, snd_con);
	Mat F = Pcv4::solve_dlt(A);
	Pcv4::forceSingularity(F);
	Pcv4::decondition(T_fst, T_snd, F);
	return F;
}

// solve homogeneous equation system by usage of SVD
/*
A			the design matrix
return		the estimated fundamental matrix
*/
Mat Pcv4::solve_dlt(Mat& A){
	// TO DO !!!
	Mat F1 = Mat(3, 3, CV_32FC1);
	cv::SVD svd(A, SVD::FULL_UV);
	svd.vt.row(8).colRange(0, 3).copyTo(F1.row(0));
	svd.vt.row(8).colRange(3, 6).copyTo(F1.row(1));
	svd.vt.row(8).colRange(6, 9).copyTo(F1.row(2));
	return F1;
}

// decondition a fundamental matrix that was estimated from conditioned point clouds
/*
T_fst	conditioning matrix of first set of points
T_snd	conditioning matrix of second set of points
F	conditioned fundamental matrix that has to be un-conditioned (in-place)
*/
void Pcv4::decondition(Mat& T_fst, Mat& T_snd, Mat& F){
	F = T_snd.t() * F * T_fst;
}

// define the design matrix as needed to compute fundamental matrix
/*
fst		first set of points
snd		second set of points
return		the design matrix to be computed
*/
Mat Pcv4::getDesignMatrix_fundamental(Mat& fst, Mat& snd){
	// TO DO !!!
	Mat A = Mat(fst.cols, 9, CV_32FC1);
	//Mat temp = Mat::ones(1,9,CV_32FC1);
	for (int i = 0; i < fst.cols; i++) {
		A.at<float>(i, 0) = fst.at<float>(0, i) * snd.at<float>(0, i);
		A.at<float>(i, 1) = fst.at<float>(1, i) * snd.at<float>(0, i);
		A.at<float>(i, 2) = snd.at<float>(0, i);
		A.at<float>(i, 3) = fst.at<float>(0, i) * snd.at<float>(1, i);
		A.at<float>(i, 4) = fst.at<float>(1, i) * snd.at<float>(1, i);
		A.at<float>(i, 5) = snd.at<float>(1, i);
		A.at<float>(i, 6) = fst.at<float>(0, i);
		A.at<float>(i, 7) = fst.at<float>(1, i);
		A.at<float>(i, 8) = 1;
	}//end of for(i)
	return A;
}

// apply transformation
/*
H			matrix representing the transformation
geomObj		matrix with input objects (one per column)
type		the type of the geometric object (for now: only point and line)
return		transformed objects (one per column)
*/
Mat Pcv4::applyH(Mat& geomObj, Mat& H, string type){
	// TO DO !!!
	// if object is a point
	if (type.compare("point") == 0) {
		return H*geomObj;
	}
	// if object is a line
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
return		the condition matrix (already allocated)
*/
Mat Pcv4::getCondition2D(Mat& p){
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

// enforce rank of 2 on fundamental matrix
/*
F	the matrix to be changed
*/
void Pcv4::forceSingularity(Mat& F){
	// TO DO !!!

	cv::SVD svdF(F, cv::SVD::FULL_UV);
	Mat d = Mat::zeros(3, 3, CV_32FC1);
	d.at<float>(0, 0) = svdF.w.at<float>(0, 0);
	d.at<float>(1, 1) = svdF.w.at<float>(1, 0);
	d.at<float>(2, 2) = 0;
	F = svdF.u*d*svdF.vt;
}

// draw epipolar lines into both images
/*
img1	structure containing fst image
img2	structure containing snd image
p_fst	first point set
p_snd	second point set
F	fundamental matrix
*/
void Pcv4::visualize(struct winInfo& img1, struct winInfo& img2, Mat& p_fst, Mat& p_snd, Mat& F){
	Mat l1, l2;
	Mat a, b;
	for (int i = 0; i < p_fst.cols; i++) {
		l2 = F.t() * p_snd.col(i);
		Pcv4::drawEpiLine(img1.img, l2.at<float>(0, 0), l2.at<float>(1, 0), l2.at<float>(2, 0));
		l1 = F * p_fst.col(i);
		Pcv4::drawEpiLine(img2.img, l1.at<float>(0, 0), l1.at<float>(1, 0), l1.at<float>(2, 0));
	}//End of for(i)
	namedWindow("Epipolar lines on the first image", WINDOW_NORMAL);
	imshow("Epipolar lines on the first image", img1.img);
	namedWindow("Epipolar lines on the second image", WINDOW_NORMAL);
	imshow("Epipolar lines on the second image", img2.img);
	cv::imwrite("First_Image_With_Epipolar_Lines.png", img1.img);
	cv::imwrite("Second_Image_With_Epipolar_Lines.png", img2.img);
	waitKey(0);
}

// calculate geometric error of estimated fundamental matrix
/*
p_fst		first set of points
p_snd		second set of points
F		fundamental matrix
return		geometric error
*/
double Pcv4::getError(Mat& p_fst, Mat& p_snd, Mat& F){
	// TO DO !!!
	double numerator, denominator, d = 0;//d: symetric epipolar distance
	cout << "\n F is:" << F;
	Mat temp11 = Mat(1, 1, CV_32FC1);
	Mat temp31 = Mat(3, 1, CV_32FC1);
	int N = p_fst.cols;// number of pair points
	Mat x_r = Mat(3, 1, CV_32FC1);//poistion in the right hand side image
	Mat x_l = Mat(3, 1, CV_32FC1);//poistion in the left hand side image
	for (int i = 0; i < N; i++) {
		x_l = p_fst.col(i);
		x_r = p_snd.col(i);
		temp11 = (x_r.t() * F * x_l);
		numerator = cv::pow(temp11.at<float>(0, 0), 2);
		denominator = 0;
		temp31 = F*x_l;
		temp31.at<float>(2, 0) = 0;
		denominator = pow(norm(temp31), 2);
		temp31 = F.t()*x_r;
		temp31.at<float>(2, 0) = 0;
		denominator = denominator + pow(norm(temp31), 2);
		d = d + numerator / denominator;
	}// end of for(i)
	return d / N;
}
/* *****************************
  GIVEN FUNCTIONS
***************************** */

// function loads the input image, calls processing function, and saves result
/*
fname	path to input image
*/
void Pcv4::run(string img1Path, string img2Path){

   // titles of some windows
    string winName1 = string ("First image");
    string winName2 = string ("Second image");

    // load images
    Mat fstImage = imread(img1Path);
    Mat sndImage = imread(img2Path);
    
    if ( (!fstImage.data) || (!sndImage.data)){
	cerr << "ERROR: Could not load images" << endl;
	cerr << "Press enter to continue..." << endl;
	cin.get();
	exit(-2);
    }

    // fuse image data and window title
    struct winInfo fst;
    fst.img = fstImage.clone();
    fst.name = winName1;    
    struct winInfo snd;
    snd.img = sndImage.clone();
    snd.name = winName2;
   
    // get corresponding points within the two images
    // start with one point within the first image, then click on corresponding point in second image
    Mat p_fst, p_snd;
    int numberOfPointPairs = getPoints(fst, snd, p_fst, p_snd);
    
    // just some putput
    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in first image:" << endl;
    cout << p_fst << endl;
    cout << endl << "Points in second image:" << endl;
    cout << p_snd << endl;
    
    // calculate fundamental matrix
    Mat F = getFundamentalMatrix(p_fst, p_snd);

    // visualize epipolar lines
    visualize(fst, snd, p_fst, p_snd, F);

    // calculate geometric error
    double err = getError(p_fst, p_snd, F);
    cout << "Geometric error: " << err << endl;
	cin.get();

}

// draw line given in homogeneous representation into image
/*
img	the image
a,b,c	the line parameters
*/
void Pcv4::drawEpiLine(Mat& img, double a, double b, double c){

  // calculate intersection with image borders
  Point p1 = Point(-c/a, 0);						// schnittpunkt mit unterer bildkante (x-achse)
  Point p2 = Point(0, -c/b);						// schnittpunkt mit linker bildkante (y-achse)
  Point p3 = Point((-b*(img.rows-1)-c)/a, img.rows-1);		// schnittpunkt mit oberer bildkante
  Point p4 = Point(img.cols-1, (-a*(img.cols-1)-c)/b);		// schnittpunkt mit rechter bildkante
  
  // check start and end points
  Point startPoint, endPoint, cur_p;
  startPoint.x = startPoint.y = endPoint.x = endPoint.y = 0;
  bool set_start = false;
  for(int p=0; p<4; p++){
    switch(p){
      case 0: cur_p = p1; break;
      case 1: cur_p = p2; break;
      case 2: cur_p = p3; break;
      case 3: cur_p = p4; break;
    }
    if ( (cur_p.x >= 0) && (cur_p.x < img.cols) && (cur_p.y >= 0) && (cur_p.y < img.rows) ){
      if (!set_start){
	startPoint = cur_p;
	set_start = true;
      }else{
	endPoint = cur_p;
      }
    }
  }
  
  // draw line
  line(img, startPoint, endPoint, Scalar(0,0,255), 1);

}

// mouse call back to get points and draw circles
/*
event	specifies encountered mouse event
x,y	position of mouse pointer
flags	not used here
param	a struct containing used IplImage and window title
*/
void getPointsCB(int event, int x, int y, int flags, void* param){

  // cast to structure
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
fst		structure containing fst image
snd		structure containing image that has to be snded
p_fst		points within the fst image (to be defined by this method)
p_snd	points within the second image (to be defined by this method)
*/
int Pcv4::getPoints(struct winInfo& fst, struct winInfo& snd, Mat& p_fst, Mat& p_snd){
  
    // show input images and install mouse callback
    namedWindow( fst.name.c_str(), 0 );
    imshow( fst.name.c_str(), fst.img );
    setMouseCallback(fst.name.c_str(), getPointsCB, (void*)(&fst));
    namedWindow( snd.name.c_str(), 0 );
    imshow( snd.name.c_str(), snd.img );
    setMouseCallback(snd.name.c_str(), getPointsCB, (void*)(&snd));
    // wait until any key was pressed
    waitKey(0);

   /* fst.pointList.clear();
    snd.pointList.clear();
    fst.pointList.push_back(Point2f(67,18));
    snd.pointList.push_back(Point2f(5,18));

    fst.pointList.push_back(Point2f(215,22));
    snd.pointList.push_back(Point2f(161,22));

    fst.pointList.push_back(Point2f(294,74));
    snd.pointList.push_back(Point2f(227,74));

    fst.pointList.push_back(Point2f(100,85));
    snd.pointList.push_back(Point2f(41,85));

    fst.pointList.push_back(Point2f(187,115));
    snd.pointList.push_back(Point2f(100,116));

    fst.pointList.push_back(Point2f(281,153));
    snd.pointList.push_back(Point2f(220,152));

    fst.pointList.push_back(Point2f(303,194));
    snd.pointList.push_back(Point2f(237,195));

    fst.pointList.push_back(Point2f(162,225));
    snd.pointList.push_back(Point2f(74,225));*/
    
    // allocate memory for point-lists (represented as matrix)
    p_fst = Mat(3, fst.pointList.size(), CV_32FC1);
    p_snd = Mat(3, snd.pointList.size(), CV_32FC1);
    int n=0;	// number of point pairs
    // read points from global variable, transform them into homogeneous coordinates
    for(vector<Point2f>::iterator p1 = fst.pointList.begin(), p2 = snd.pointList.begin(); p1 != fst.pointList.end(); p1++, p2++){
		p_fst.at<float>(0, n) = p1->x;
		p_fst.at<float>(1, n) = p1->y;
		p_fst.at<float>(2, n) = 1;
		p_snd.at<float>(0, n) = p2->x;
		p_snd.at<float>(1, n) = p2->y;
		p_snd.at<float>(2, n) = 1;
		n++;
    }

    // close windows
    destroyWindow( fst.name.c_str() );
    destroyWindow( snd.name.c_str() );

    return n;

}
