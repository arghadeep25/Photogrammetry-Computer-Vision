//============================================================================
// Name        : main.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : only calls processing and test routines
//============================================================================

#include <iostream>

#include "Pcv2.h"

using namespace std;

// usage: path to image in argv[1]
// main function. loads and saves image
int main(int argc, char** argv) {

	// will contain path to the input image (taken from argv[1])
	string fnameBase, fnameLeft, fnameRight;

    // check if image paths are defined
    if (argc != 4){
		cout << "Usage: pcv2 <path to base image> <path to 2nd image> <path to 3rd image>" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		return -1;
    }else{
	    // if yes, assign it to variable fname
	    fnameBase = argv[1];
	    fnameLeft = argv[2];
	    fnameRight = argv[3];
	}
	
	// construct processing object
	Pcv2 pcv2;

	// run some test routines
	pcv2.test();

	// start processing
	pcv2.run(fnameBase, fnameLeft, fnameRight);

	cout << "Press enter to continue..." << endl;
	cin.get();

	return 0;

}
