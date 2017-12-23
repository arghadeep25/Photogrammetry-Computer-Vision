//============================================================================
// Name        : main.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : only calls processing and test routines
//============================================================================

#include <iostream>

#include "Pcv3.h"

using namespace std;

// usage: path to image in argv[1]
// main function. loads and saves image
int main(int argc, char** argv) {

	// will contain path to input image (taken from argv[1])
	string imgPath, pointPath;

    // check if image paths are defined
    if (argc != 3){
		cerr << "Usage: pcv3 <path to calibration image> <path to object-point file>" << endl;
		cerr << "Press enter to continue..." << endl;
		cin.get();
		return -1;
    }else{
	    // if yes, assign it to variable fname
	    imgPath = argv[1];
	    pointPath = argv[2];
	}
	
	// construct processing object
	Pcv3 pcv3;

	// run test routines
	pcv3.test();

	// start processing
	pcv3.run(imgPath, pointPath);

	cout << "Press enter to continue..." << endl;
	cin.get();

	return 0;

}
