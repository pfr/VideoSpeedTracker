//                  Copyright Paul Reynolds, Locust Avenue, Charlottesville, Va,  2016
//                                     All rights reserved.

//                                     License Agreement
//                                For VideoSpeedTracker (VST)
//                                 (3 - clause BSD License)

// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following
// conditions are met :

// 1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
//      in the documentation and / or other materials provided with the distribution.
// 3) Neither the name of the copyright holder nor the names of the contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.

// This software is provided by the copyright holder and contributors “as is” and any express or implied warranties, including,
// but not limited to, the implied warranties of merchantability and fitness for a particular purpose are disclaimed.In no event
// shall copyright holders or contributors be liable for any direct, indirect, incidental, special, exemplary, or consequential
// damages(including, but not limited to, procurement of substitute goods or services; loss of use, data, or profits; or business
// interruption) however caused and on any theory of liability, whether in contract, strict liability, or tort(including negligence
// or otherwise) arising in any way out ofthe use of this software, even if advised of the possibility of such damage.
//

//  Run this code on output highlights file of VideoSpeedTracker.  The program produces highlights of highlights as edited by the user.
// Output is placed in subdirectory "forPosting"  with the prefix "for_post" prepended to the input file name.


#include <opencv\cv.h>
#include "opencv2\highgui\highgui.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#define CVBlack 0,0,0
#define CVCyan 255,255,153

using namespace std;
using namespace cv;

VideoCapture hiLiteVideoIn;  //Input hiLites video
VideoWriter hiLiteVideoOut; // For writing highlights...the edited Scofflaws
Mat frames[200];
const int maxFramePtr = 199;
int framePTR = 0;
ifstream filesList;
string fileName;  // Name of avi file currently being processed.
int leftSide = 320;  // These values are very specific to the setup used in program trafficReports.   ***********************************
int top = 120;  // It woould be better to have these passed in as parameters from the trafficReports program.   **********************

string inLine, lhs, rhs;
string dataPathPrefix;


bool getSides(string inLine){
	string tempLHS, tempRHS;
	istringstream configLine(inLine);
	getline(configLine, tempLHS, '=');
	getline(configLine, tempRHS, '#');
	lhs = tempLHS;
	rhs = tempRHS;
	return true;
}

string trim(string toBeTrimmed){
	int first = 0;
	while (toBeTrimmed.substr(first, 1) == " ") first++;
	int last = toBeTrimmed.length() - 1;
	while (toBeTrimmed.substr(last, 1) == " ") last--;
	return toBeTrimmed.substr(first, (last - first) + 1);
}

void replay(int inFramePTR, int inDelay){
	for (int i = 0; i < inFramePTR; i++){
		imshow("Next Frame", frames[i]);
		switch (waitKey(inDelay));
	}
}

bool blackFrame(Mat inFrame){
	string lineIn;
	Scalar res = mean(inFrame);
	return (res[0] < 0.1 && res[1] < 0.1 && res[2] < 0.1);
}

int main(){
// Get data path prefix from ProcessHiLites.cfg

	vector<string> neighborhoods;
	string answer;
	int lineNo = 0;
	string chosenNeighborhood = "";
	ifstream hoodsIn("neighborhoods.cfg");
	if (hoodsIn.good()){
		getline(hoodsIn, inLine);
		if (!hoodsIn){
			cout << "Unexpected error in neighborhood config file.  Aborting." << endl;
			return false;
		}
		if (!getSides(inLine)){
			cout << "dataPathPrefix in neighborhoods.cfg file has syntax error.  Aborting." << endl;
			return false;
		}
		lhs = trim(lhs);
		rhs = trim(rhs);
		if (lhs != "dataPathPrefix"){
			cout << "Expected dataPathPrefix in neighborhoods.cfg.  Not found. Aborting." << endl;
			return false;
		}
		dataPathPrefix = rhs;
		cout << "dataPathPrefix = " << dataPathPrefix << endl;
		cout << "Neighborhoods: " << endl;
		while (!hoodsIn.eof()){
			getline(hoodsIn, inLine);
			if (!hoodsIn){
				cout << "Unexpected error in neighborhoods config file. Aborting." << endl;
				hoodsIn.close();
				return false;;
			}
			else if (!getSides(inLine)){
				cout << "getSides() failed in config reader.  Check VST.cfg syntax.  Aborting." << endl;
				hoodsIn.close();
				return false;
			}
			else{
				cout << "  " << ++lineNo << ") " << trim(lhs) << endl;
				neighborhoods.push_back(trim(rhs));
			}

		} // while

		if (neighborhoods.size() <= 0){
			cout << "Empty neighborhoods.cfg file.  Looking for VST.cfg next." << endl;
		}
		else {
			cout << "Choose a neighborhood by number [1]:";
			getline(cin, answer);
			int index;
			if (!answer.empty()) index = stoi(answer);
			if (index < 1 || index > neighborhoods.size())
				index = 1;
			chosenNeighborhood = neighborhoods.at(index - 1);
			cout << endl;
		}
		cout << "Chosen neighborhood is: " << chosenNeighborhood << endl;
		dataPathPrefix = dataPathPrefix + "\\" + chosenNeighborhood + "Data";
		hoodsIn.close();
	}
	else
		cout << "No neighborhoods.cfg file.  Looking for VST.cfg next." << endl;





	string lhsString[3] = {
		"dataPathPrefix",
		"CropLeft",
		"CropTop"
	};


	ifstream configIn;
	if (chosenNeighborhood == ""){
		//		cout << "Trying to open file ProcessHiLites.cfg" << endl;
		configIn.open("ProcessHiLites.cfg");
		if (!configIn.good()){
			cout << "Can't open ProcessHiLites.cfg." << endl;
			return false;
		}
	}
	else {
		//		cout << "Trying to open file " << dataPathPrefix << "\\" << chosenNeighborhood << "HiLites.cfg" << endl;
		configIn.open(dataPathPrefix + "\\" + chosenNeighborhood + "HiLites.cfg");
		if (!configIn.good()){
			cout << "Can't open " << chosenNeighborhood << "HiLites.cfg" << endl;
			return false;
		}
	}
	lineNo = 0;
	while (!configIn.eof()){
		getline(configIn, inLine);
		if (!configIn){
			cout << "Unexpected error in config file.  Spinning for ctrl-c." << endl;
			while (1){}
		}
		if (getSides(inLine)){
			lhs = trim(lhs);
			rhs = trim(rhs);
			if (lhs != lhsString[lineNo]){
				cout << "Just read LHS doesn't match anything: <" << lhs << ">.  Spinning for ctrl-c." << endl;
				while (1){}
			}
			switch (lineNo){
			case 0:
				if (chosenNeighborhood == "")
					dataPathPrefix = rhs;
//				cout << "dataPathPrefix = " << dataPathPrefix << endl;
				break;
			case 1:
				leftSide = stoi(rhs);
//				cout << "leftSide = " << leftSide << endl;
				break;
			case 2:
				top = stoi(rhs);
//				cout << "top = " << top << endl << endl;
				cout << endl;
				break;
			default:
				if (lineNo > 2){
					cout << "Too many lines in config file.  Spinning for ctrl-c." << endl;
					while (1){}
				}
			} // switch
			lineNo++;
		}
		else{
			cout << "getSides() failed in config reader.  Check VST.cfg syntax.   Spinning for ctrl-c. " << endl;
			configIn.close();
			while (1){}
		}

	} // while not eof in config file

	string dirPath = dataPathPrefix + "\\HiLites";
	string sysString = "dir " + dirPath + "\\*.avi /b > " + dirPath + "\\files.txt";
	const char * c = sysString.c_str();
	system(c); // copy the file names from the chosen directory to file "files.txt" in the same directory.
	string yesNo = "n";
	// Get file user wants
	while (yesNo == "n"){
		filesList.open(dirPath + "\\files.txt");
		while (getline(filesList, fileName)){
			cout << "Want the file " << fileName << "  (y/n) [n]: ";
			getline(cin, yesNo);
			if (yesNo == "y") break;  // User has chosen the file
			else yesNo = "n";
		}
		filesList.close();
	}

	string fullName = dirPath + "\\" + fileName;
//	cout << "Fullname is: <" << fullName << "> " << endl;
	hiLiteVideoIn.open(fullName);
	
	if (!hiLiteVideoIn.isOpened()){
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return 0;
	}

	if (hiLiteVideoIn.get(CV_CAP_PROP_POS_FRAMES) <= hiLiteVideoIn.get(CV_CAP_PROP_FRAME_COUNT)){
		hiLiteVideoIn.read(frames[0]);
// NOTE: the following operation crops a 640 x 360 rectangle out of the middlle of input frame.  Dependng on the width of your
// speed zone, this may not work for you...parts of your speed zone may be cropped off the ends.
		rectangle(frames[0], Rect(Point(leftSide, top), Point(leftSide + 639, top + 359)), Scalar(CVCyan), 4);
		imshow("Next Frame", frames[0]);
		switch (waitKey(20));
		yesNo = "n";
		cout << "Is this cropping OK for saved captures? (y/n) [y]: ";
		getline(cin, yesNo);
		if (yesNo == "n") return -1; // Coould modify this to move cropping rectangle around until user happy.
	}

	hiLiteVideoIn.release();
	switch (waitKey(20));
	hiLiteVideoIn.open(fullName);
	if (!hiLiteVideoIn.isOpened()){
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return 0;
	}

	fullName = dirPath + "\\forPosting\\forPost_" + fileName;
	hiLiteVideoOut.open(fullName, -1, hiLiteVideoIn.get(CV_CAP_PROP_FPS), Size(640, 360), true);
	if (!hiLiteVideoOut.isOpened()){
		cout << "ERROR Opening output file\n";
		getchar();
		return 0;
	}


	bool atVideoEnd = false;
	string response;

	while (hiLiteVideoIn.get(CV_CAP_PROP_POS_FRAMES) < hiLiteVideoIn.get(CV_CAP_PROP_FRAME_COUNT)){

		framePTR = 0;
		bool atVehicleClipEnd = false;
		while (!atVehicleClipEnd){
			if (hiLiteVideoIn.get(CV_CAP_PROP_POS_FRAMES) <= hiLiteVideoIn.get(CV_CAP_PROP_FRAME_COUNT)){
				hiLiteVideoIn.read(frames[framePTR]);
				if (!blackFrame(frames[framePTR])){  //  If this is a black frame, keep it but don't show it
					imshow("Next Frame", frames[framePTR]);
					switch (waitKey(20));
				}
				else {
					atVehicleClipEnd = true;
				}
				framePTR++;
			}
			else{
				atVehicleClipEnd = true;
				atVideoEnd = true;
			}
		} // while reading frames for current vehicle


		if (!atVideoEnd){
			bool noAction = true;
			while (noAction){
				cout << "[K]eep vehicle,  [D]elete vehicle,  [R]eplay,  [S]low replay,  [Q]uit: ";
				getline(cin, response);
				if (response == "d"){  // "d"   delete up to and including last frame read.
					framePTR = 0;
					noAction = false;
				}
				else if (response == "k"){  // "k" keep vehicle in hiLites
					for (int i = 0; i < 15; i++)
						hiLiteVideoOut.write(frames[0](Rect(leftSide, top, 640, 360)));
					for (int i = 0; i < framePTR-1; i++)
						hiLiteVideoOut.write(frames[i](Rect(leftSide,top,640,360)));
					for (int i = 0; i < 20; i++)
						hiLiteVideoOut.write(frames[framePTR-2](Rect(leftSide, top, 640, 360)));
					hiLiteVideoOut.write(frames[framePTR - 1](Rect(leftSide, top, 640, 360)));
					framePTR = 0;
					noAction = false;
				}
				else if (response == "r"){  // "r"  replay normal speed
					replay(1, 250);
					replay(framePTR-1, 20);
				}
				else if (response == "s"){  // "s"  replay slow speed
					replay(framePTR-1, 250);
				}
				else if (response == "q"){  // "q"  quit.
					cout << "quitting" << endl;
					noAction = false;
				}
			} // while no action
		}  // If not at video end

		if (response == "q")
			break;          //  Stop reading any frames and finish up.
	} // while frames remain to be read

	if (framePTR > 0){
		cout << "There are " << framePTR << " frames left in buffer.  Write them out? [y|n] <y>: ";
		getline(cin, response);
		if (!response.empty() && response == "y")
			for (int i = 0; i < framePTR; i++)
				hiLiteVideoOut.write(frames[i](Rect(leftSide, top, 640, 360)));
	}

	hiLiteVideoIn.release();
	hiLiteVideoOut.release();

	return 0;
}

