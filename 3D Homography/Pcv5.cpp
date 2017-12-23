//============================================================================
// Name        : Pcv5.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : Triangulation
//============================================================================

#include "Pcv5.h"

// compute fundamental matrix
/*
fst	first set of points
snd	second set of points
return	the estimated fundamental matrix
*/
Mat Pcv5::getFundamentalMatrix(Mat& fst, Mat& snd){
	// TODO !!!
	Mat T_fst = Pcv5::getCondition2D(fst);
	Mat T_snd = Pcv5::getCondition2D(snd);
	Mat fst_con = applyH(fst, T_fst, "point");
	Mat snd_con = applyH(snd, T_snd, "point");
	Mat A = Pcv5::getDesignMatrix_fundamental(fst_con, snd_con);
	Mat F = Pcv5::solve_dlt(A);
	Pcv5::forceSingularity(F);
	Pcv5::decondition_fundamental(T_fst, T_snd, F);
	return F;
}

// solve homogeneous equation system by usage of SVD
/*
A			the design matrix
return		the estimated fundamental matrix
*/
Mat Pcv5::solve_dlt(Mat& A){
	// TODO !!!
	cv::SVD svd(A.t()*A, SVD::FULL_UV);
	if (A.cols == 4)
		return svd.vt.row(3).t();
	else
		return svd.vt.row(A.cols - 1).reshape(0, sqrt(A.cols));
}

// decondition a fundamental matrix that was estimated from conditioned point clouds
/*
T_fst	conditioning matrix of first set of points
T_snd	conditioning matrix of second set of points
F	conditioned fundamental matrix that has to be un-conditioned (in-place)
*/
void Pcv5::decondition_fundamental(Mat& T_fst, Mat& T_snd, Mat& F){
	// TODO !!!
	F = T_snd.t() * F * T_fst;
}

// define the design matrix as needed to compute fundamental matrix
/*
fst		first set of points
snd		second set of points
return		the design matrix to be computed
*/
Mat Pcv5::getDesignMatrix_fundamental(Mat& fst, Mat& snd){
	// TODO !!!
	Mat A = Mat(fst.cols, 9, CV_32FC1);
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

// define the design matrix as needed to compute fundamental matrix
/*
fst	first set of points
snd	second set of points
A	the design matrix to be computed
*/
Mat Pcv5::getDesignMatrix_homography3D(Mat& fst, Mat& snd){
	// TODO !!!
	int n = fst.cols;
	Mat A = Mat::zeros(3 * n, 16, CV_32FC1);
	double u1, v1, q1, w1, u, v, q, w;
	Mat temp;
	for (int i = 0; i < n; i++) {
		// Ground
		u = fst.at<float>(0, i);
		v = fst.at<float>(1, i);
		q = fst.at<float>(2, i);
		w = fst.at<float>(3, i);

		// Model
		u1 = snd.at<float>(0, i);
		v1 = snd.at<float>(1, i);
		q1 = snd.at<float>(2, i);
		w1 = snd.at<float>(3, i);

		temp = (Mat_<float>(1, 16) << -w1*u, -w1*v, -w1*q, -w1*w, 0, 0, 0, 0, 0, 0, 0, 0, u1*u, u1*v, u1*q, u1*w);
		temp.copyTo(A.row(3 * i));
		temp = (Mat_<float>(1, 16) << 0, 0, 0, 0, -w1*u, -w1*v, -w1*q, -w1*w, 0, 0, 0, 0, v1*u, v1*v, v1*q, v1*w);
		temp.copyTo(A.row(3 * i + 1));
		temp = (Mat_<float>(1, 16) << 0, 0, 0, 0, 0, 0, 0, 0, -w1*u, -w1*v, -w1*q, -w1*w, q1*u, q1*v, q1*q, q1*w);
		temp.copyTo(A.row(3 * i + 2));
	}
	return A;
}

// apply transformation
/*
geomObj		matrix with input objects (one per column)
H			matrix representing the transformation
type		the type of the geometric object (for now: only point and line)
return		transformed objects (one per column)
*/
Mat Pcv5::applyH(Mat& geomObj, Mat& H, string type){
	// TODO !!!
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
Mat Pcv5::getCondition2D(Mat& p){
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
	return Mat();
}

// get the conditioning matrix of given 3D points
/*
p	the points as matrix
T	the condition matrix (already allocated)
*/
Mat Pcv5::getCondition3D(Mat& p){
	// TODO !!!
	Mat one = Mat::ones(1, p.cols, CV_32FC1);
	double tx = p.row(0).dot(one) / p.cols;
	double ty = p.row(1).dot(one) / p.cols;
	double tz = p.row(2).dot(one) / p.cols;
	double sx = (abs(tx*one - p.row(0))).dot(one) / p.cols;
	double sy = (abs(ty*one - p.row(1))).dot(one) / p.cols;
	double sz = (abs(tz*one - p.row(2))).dot(one) / p.cols;
	Mat T = Mat::eye(4, 4, CV_32S);
	if (sx != 0 && sy != 0 && sz != 0) {
		T = (Mat_<float>(4, 4) << 1 / sx, 0, 0, -tx / sx, 0, 1 / sy, 0, -ty / sy, 0, 0, 1 / sz, -tz / sz, 0, 0, 0, 1);
	}
	return T;
}

// enforce rank of 2 on fundamental matrix
/*
F	the matrix to be changed
*/
void Pcv5::forceSingularity(Mat& F){
	// TODO !!!
	cv::SVD svd(F, SVD::FULL_UV);
	Mat temp = Mat::zeros(3, 3, CV_32FC1);
	temp.at<float>(0, 0) = svd.w.at<float>(0, 0);
	temp.at<float>(1, 1) = svd.w.at<float>(1, 0);
	F = svd.u * temp * svd.vt;
}

// calculates epipols from fundamental matrix
/*
F	the fundamental matrix
e1	first epipol
e2	second epipol
*/
void Pcv5::getEpipols(Mat& F, Mat& e1, Mat& e2){
	// TODO !!!
	cv::SVD svd_F(F, SVD::FULL_UV);
	e1 = svd_F.vt.row(2).t();
	e2 = svd_F.u.col(2);
}

// generates skew matrix from vector
/*
e		given vector
return		skew matrix
*/
Mat Pcv5::makeSkewMatrix(Mat& e){
	// TODO !!!
	Mat a = Mat::zeros(3, 3, CV_32FC1);
	a.at<float>(0, 1) = -1 * e.at<float>(2, 0);
	a.at<float>(1, 0) = e.at<float>(2, 0);
	a.at<float>(0, 2) = e.at<float>(1, 0);
	a.at<float>(2, 0) = -1 * e.at<float>(1, 0);
	a.at<float>(1, 2) = -1 * e.at<float>(0, 0);
	a.at<float>(2, 1) = e.at<float>(0, 0);
	return a;
}

// generates 2 projection matrices by using fundamental matrix
/*
F	the fundamental matrix
P1	first projection matrix (standard P matrix)
P2	second projection matrix based on F
*/
void Pcv5::defineCameras(Mat& F, Mat& P1, Mat& P2){
	// TODO !!!
	P1 = Mat::eye(3, 4, CV_32FC1);
	Mat e1 = Mat(3, 1, CV_32FC1);
	Mat e2 = Mat(3, 1, CV_32FC1);
	Pcv5::getEpipols(F, e1, e2);
	Mat temp = Mat(3, 3, CV_32FC1);
	temp = Pcv5::makeSkewMatrix(e2) * F;
	hconcat(temp, e2, P2);
}

// triangulates given set of image points based on projection matrices
/*
P1	projection matrix of first image
P2	projection matrix of second image
x1	image point set of first image
x2	image point set of second image
return	triangulated object points
*/
Mat Pcv5::linearTriangulation(Mat& P1, Mat& P2, Mat& x1, Mat& x2){
	// TODO !!!
	Mat temp = Mat(1, 4, CV_32FC1);
	int n = x1.cols;
	Mat X = Mat(4, n, CV_32FC1);
	Mat A = Mat(4, 4, CV_32FC1);
	for (int i = 0; i < n; i++) {
		temp = x1.at<float>(0, i)*P1.row(2) - P1.row(0);
		temp.copyTo(A.row(0));
		temp = x1.at<float>(1, i)*P1.row(2) - P1.row(1);
		temp.copyTo(A.row(1));
		temp = x2.at<float>(0, i)*P2.row(2) - P2.row(0);
		temp.copyTo(A.row(2));
		temp = x2.at<float>(1, i)*P2.row(2) - P2.row(1);
		temp.copyTo(A.row(3));
		solve_dlt(A).copyTo(X.col(i));
	}
	return X;
}

// computes 3D homography
/*
X1	first set of points
X2	second set of points
H	computed homography
*/
Mat Pcv5::homography3D(Mat& X1, Mat& X2){
	// TODO !!!
	Mat T_X1 = Pcv5::getCondition3D(X1);
	Mat X1_con = applyH(X1, T_X1, "point");
	Mat T_X2 = Pcv5::getCondition3D(X2);
	Mat X2_con = applyH(X2, T_X2, "point");
	Mat A = getDesignMatrix_homography3D(X1_con, X2_con);
	Mat H = Pcv5::solve_dlt(A);
	Pcv5::decondition_homography3D(T_X1, T_X2, H);
	H = H / H.at<float>(3, 3);
	return H;
}

// decondition a homography that was estimated from conditioned point clouds
/*
T_to	conditioning matrix of first set of points
T_from	conditioning matrix of second set of points
H	conditioned homography that has to be un-conditioned (in-place)
*/
void Pcv5::decondition_homography3D(Mat& T_to, Mat& T_from, Mat& H){
	// TODO !!!
	H = T_from.inv()*H*T_to;
}

/* *****************************
  GIVEN FUNCTIONS
***************************** */

// function loads input image, calls processing function, and saves result
/*
fname	path to input image
*/
void Pcv5::run(string imgPoints, string ctrPoints){

   // get corresponding points within the two images
    Mat x1, x2;
    int numberOfPointPairs = readMatchingPoints(imgPoints, x1, x2);
    
    // just some putput
    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    
    // calculate fundamental matrix
    Mat F = getFundamentalMatrix(x1, x2);

    cout << endl << "Fundamental matrix: " << F << endl;

    // define projection matrices
    Mat P1, P2;
    defineCameras(F, P1, P2);
   
    cout << endl << "Camera 1: " << P1 << endl;
    cout << endl << "Camera 2: " << P2 << endl;
    
    // linear triangulation of image points
    // resulting projective reconstruction
    Mat X = linearTriangulation(P1, P2, x1, x2);

    // save reconstructed points to file
    savePointList("projectiveReconstruction.asc", X);
    
    // read control points
    // clean re-use of x1- and x2-matrices
    Mat Xm;
    numberOfPointPairs = readControlPoints(ctrPoints, x1, x2, Xm);

    cout << endl << "Control points:" << endl;
    cout << "x1: " << x1 << endl;
    cout << "x2: " << x2 << endl;
    cout << "Xm: " << Xm << endl;

    // linear triangulation of image points
    Mat Xp = linearTriangulation(P1, P2, x1, x2);
    cout << "Reconstructed control points: " << Xp << endl;

    // Transform projective reconstructed points to euclidian reconstruction
    Mat H = homography3D(Xp, Xm);
    Mat X_final = applyH(X, H, "point");
    
    // save reconstructed points to file
    savePointList("euclidianReconstruction_new.asc", X_final);

}

// saves point list to file
/*
fname		file name
points		matrix of points
*/
void Pcv5::savePointList(string fname, Mat& points){

  // open file for write
  fstream file(fname.c_str(), ios::out);
  if(!file){
      cerr << "ERROR: cannot open file " << fname << endl;
	  cerr << "Press enter to continue..." << endl;
	  cin.get();
      exit(-1);
  }
  
  // if homogeneous points: norm and write points
  if (points.rows == 4){
      for(int i=0; i<points.cols; i++){
		file << points.at<float>(0, i)/points.at<float>(3, i) << "," << points.at<float>(1, i)/points.at<float>(3, i) << "," << points.at<float>(2, i)/points.at<float>(3, i) << endl;
      }
  }
  // if euclidian points: write points
  if (points.rows == 3){
      for(int i=0; i<points.cols; i++){
		file << points.at<float>(0, i) << "," << points.at<float>(1, i) << "," << points.at<float>(2, i) << endl;
      }
  }
  
  // close file
  file.close();

}

// read homologeous points from file
/*
fname	path of file containing point list
x1	pointer to matrix containing points of first image
x2	pointer to matrix containing points of second image
*/
int Pcv5::readMatchingPoints(string fname, Mat& x1, Mat& x2){

    // open file
    fstream file(fname.c_str(), ios::in);
    if (!file){
		cerr << "ERROR: Cannot open file " << fname << endl;
	    cerr << "Press enter to continue..." << endl;
	    cin.get();
        exit(-1);
    }

    // read points into list
    list<Scalar> points;
    int end, numberOfPointPairs = 0;
    string buffer;
    // read line by line
    while(getline(file, buffer)){

		// counting
		numberOfPointPairs++;
		
		// get semicolon
		end = buffer.find(';');
		// first point before semicolon
		string fst = buffer.substr(0, end);
		// second point after semicolon
		string snd = buffer.substr(end+1);
		
		// get x and y
		Scalar cur;
		end = fst.find(',');
		cur.val[0] = atof(fst.substr(0, end).c_str());
		cur.val[1] = atof(fst.substr(end+1).c_str());
		
		// get x and y
		end = snd.find(',');
		cur.val[2] = atof(snd.substr(0, end).c_str());
		cur.val[3] = atof(snd.substr(end+1).c_str());
		
		// push point pair to list
		points.push_back(cur);
    }
    
    // allocate memory for point matrices
    x1 = Mat(3, numberOfPointPairs, CV_32FC1);
    x2 = Mat(3, numberOfPointPairs, CV_32FC1);
    
    // fill point matrices
    int i=0;
    for(list<Scalar>::iterator p = points.begin(); p != points.end(); p++,i++){
		// x1
		x1.at<float>(0, i) = (*p).val[0];
		x1.at<float>(1, i) = (*p).val[1];
		x1.at<float>(2, i) = 1;
		// x2
		x2.at<float>(0, i) = (*p).val[2];
		x2.at<float>(1, i) = (*p).val[3];
		x2.at<float>(2, i) = 1;
    }
    return numberOfPointPairs;
}

// read control points from file
/*
fname	path of file containing point list
x1	pointer to matrix containing points of first image
x2	pointer to matrix containing points of second image
Xm	pointer to matrix containing object points
*/
int Pcv5::readControlPoints(string fname, Mat& x1, Mat& x2, Mat& Xm){

    // open file
    fstream file(fname.c_str(), ios::in);
    if (!file){
		cerr << "ERROR: Cannot open file " << fname << endl;
	    cerr << "Press enter to continue..." << endl;
	    cin.get();
        exit(-1);
    }

    // read points into list
    list< vector<double> > points;
    int pos1, pos2, numberOfPointPairs = 0;
    string buffer;
    // read line by line
    while(getline(file, buffer)){
      
		// counting
		numberOfPointPairs++;
		
		// get semicolons
		pos1 = buffer.find(';');
		pos2 = buffer.rfind(';');
		// first point before first semicolon
		string fst = buffer.substr(0, pos1);
		// second point between semicolons
		string snd = buffer.substr(pos1+1, pos2);
		// third point after last semicolon
		string thd = buffer.substr(pos2+1);
		
		// current point triplet
		vector<double> cur;

		// get x and y of first point
		pos1 = fst.find(',');
		cur.push_back( atof(fst.substr(0, pos1).c_str()) );
		cur.push_back( atof(fst.substr(pos1+1).c_str()) );
		// get x and y of second point
		pos1 = snd.find(',');
		cur.push_back( atof(snd.substr(0, pos1).c_str()) );
		cur.push_back( atof(snd.substr(pos1+1).c_str()) );
		// get x, y, and z of object point
		pos1 = thd.find(',');
		pos2 = thd.rfind(',');
		cur.push_back( atof(thd.substr(0, pos1).c_str()) );
		cur.push_back( atof(thd.substr(pos1+1, pos2).c_str()) );
		cur.push_back( atof(thd.substr(pos2+1).c_str()) );
		
		// push point triplet to list
		points.push_back(cur);
    }
    
    // allocate memory for point matrices
    x1 = Mat(3, numberOfPointPairs, CV_32FC1);
    x2 = Mat(3, numberOfPointPairs, CV_32FC1);
    Xm = Mat(4, numberOfPointPairs, CV_32FC1);
    
    // fill point matrices
    int i=0;
    for(list< vector<double> >::iterator p = points.begin(); p != points.end(); p++,i++){
		// x1
		x1.at<float>(0, i) = (*p).at(0);
		x1.at<float>(1, i) = (*p).at(1);
		x1.at<float>(2, i) = 1;
		// x2
		x2.at<float>(0, i) = (*p).at(2);
		x2.at<float>(1, i) = (*p).at(3);
		x2.at<float>(2, i) = 1;
		// Xm
		Xm.at<float>(0, i) = (*p).at(4);
		Xm.at<float>(1, i) = (*p).at(5);
		Xm.at<float>(2, i) = (*p).at(6);
		Xm.at<float>(3, i) = 1;
    }
    
    return numberOfPointPairs;
}
