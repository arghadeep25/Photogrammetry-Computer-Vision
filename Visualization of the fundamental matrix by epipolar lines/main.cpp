//============================================================================
// Name        : main.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : only calls processing and test routines
//============================================================================

#include <iostream>

#include "Pcv4.h"

using namespace std;

// usage: path to the images in argv[1-2]
// main function. loads and saves image
int main(int argc, char** argv) {

	// will contain path to input image (taken from argv[1])
	string img1Path, img2Path;

    // check if image paths were defined
    if (argc != 3){
		cerr << "Usage: pcv5 <path to 1st image> <path to 2nd image>" << endl;
		cerr << "Press enter to continue..." << endl;
		cin.get();
		return -1;
    }else{
	    // if yes, assign it to variable fname
	    img1Path = argv[1];
	    img2Path = argv[2];
	}
	
	// construct processing object
	Pcv4 pcv4;

	// start processing
	pcv4.run(img1Path, img2Path);

	cout << "Press enter to continue..." << endl;
	cin.get();

	return 0;

}
