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

#define CVBlue 255,0,0
#define CVGreen 0,255,0
#define CVRed 0,0,255
#define CVYellow 0,255,255
#define CVOrange 0,128,255
#define CVPurple 255,0,127
#define CVCyan 255,255,153
#define CVPink 178,102,153
#define CVWhite 255,255,255
#define CVBlack 0,0,0

using namespace std;
using namespace cv;

enum directionType {NEITHER, L2R, R2L };
enum vehicleType {UNK, cityBus, citySchoolBus, countySchoolBus};

struct frameSequenceComponents {
	Mat frames[200];  // enuf to hold one vehicle frame sequence;
	int framePTR; // index into frames[], Where the current frame being read in is
	int markerFramePTR; // index of the frame where the vehicle was centered betwqeen speed measuring posts
	string vehicleStats[8]; // The eight values read from the hiLiteStas file for this vehicle
	directionType vehicleDirection;
	int xCoordFB;
	int yCoordFB;
	vehicleType vehicleKind;


};

frameSequenceComponents vehicle[2];

VideoCapture hiLiteVideoIn;  //Input hiLites video
VideoWriter hiLiteVideoOut; // For writing highlights...the edited Scofflaws
const int maxFramePtr = 199;
int vehIndex = 0;
ifstream filesList;
ifstream hiLiteStats;
string fileName;  // Name of avi file currently being processed.
int leftSide = 320;  // These values are very specific to the setup used in program trafficReports.   ***********************************
int top = 120;  // It woould be better to have these passed in as parameters from the trafficReports program.   **********************

string inLine, lhs, rhs;
string dataPathPrefix;


//int to string helper function
string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}


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

void replay(int inFrameCount, int inDelay){
	imshow("First Frame", vehicle[vehIndex].frames[0](Rect(0, top, 1280, 360)));
	waitKey(20);
	for (int i = 0; i < inFrameCount; i++){
		imshow("Next Frame", vehicle[vehIndex].frames[i](Rect(0, top, 1280, 360)));
		waitKey(inDelay);
	}
	imshow("Marker Frame", vehicle[vehIndex].frames[vehicle[vehIndex].markerFramePTR](Rect(0, top, 1280, 360)));
	waitKey(20);
}

bool upperWhiteSquare(Mat &inFrame){
	Scalar res = mean(inFrame(Rect(0, 0, 6, 6)));
//	cout << res[0] << ",  " << res[1] << ",  " << res[2] << endl;
	return (res[0] > 240.0 && res[1] > 240.0 && res[2] > 240.0);
}


bool blackFrame(Mat &inFrame){
	Scalar res = mean(inFrame);
	return (res[0] < 0.1 && res[1] < 0.1 && res[2] < 0.1);
}


void initializeVehicle(frameSequenceComponents &inVehicle){

	inVehicle.framePTR = 0;
	inVehicle.markerFramePTR = 0;
	inVehicle.vehicleDirection = NEITHER;
	inVehicle.vehicleKind = UNK;

	return;
}

bool isVehicleKnown(frameSequenceComponents &inVehicle){
// Using input params as a guide, look through inventory of images to determine if there's a match between the
// current vehicle under consideration and any of the images in a database heading in the same direction

	return false;

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

// Determine the hiLites file the user wants to process
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

// Open the selected hiLites Video
	string fullName = dirPath + "\\" + fileName;
	hiLiteVideoIn.open(fullName);
	
	if (!hiLiteVideoIn.isOpened()){
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return 0;
	}

// Open the hiLites video stats file; entries give date, time, direction, frontbumper loc at midpoint, area and speed
// that correspond to the vehicles seen in the hiLites Video.

	string suffix = fileName.substr(8, 8);
	if (fileName.length() > 20) suffix += ("_" + fileName.substr(17, 6));

	string hiLiteStatsFileName = "hiLiteStats_" + suffix + ".csv";
	string hiLiteStatsFullPath = dataPathPrefix + "\\stats\\" + hiLiteStatsFileName;

	cout << "Trying to open:  " << hiLiteStatsFullPath << endl;

	hiLiteStats.open(hiLiteStatsFullPath);
	if (!hiLiteStats.good()){
		cout << endl << "# # # Can't open hiLiteStats file:  " << hiLiteStatsFileName << "   Must quit." << endl;
		getchar();
		return false;
	}



// Propose cropping to user (cropping doen to make output videos more viewable on the web).
	if (hiLiteVideoIn.get(CV_CAP_PROP_POS_FRAMES) <= hiLiteVideoIn.get(CV_CAP_PROP_FRAME_COUNT)){
		hiLiteVideoIn.read(vehicle[vehIndex].frames[0]);
// NOTE: the following operation crops a 640 x 360 rectangle out of the middlle of input frame.  Dependng on the width of your
// speed zone, this may not work for you...parts of your speed zone may be cropped off the ends.
		rectangle(vehicle[vehIndex].frames[0], Rect(Point(leftSide, top), Point(leftSide + 639, top + 359)), Scalar(CVCyan), 4);
		imshow("Next Frame", vehicle[vehIndex].frames[0](Rect(0, top - 5, 1280, 360 + 5)));
		waitKey(20);
		yesNo = "n";
		cout << "Is this cropping OK for saved captures? (y/n) [y]: ";
		getline(cin, yesNo);
		if (yesNo == "n") return -1; // Coould modify this to move cropping rectangle around until user happy.
	}

// Restart opening of hiLitesVideo
	hiLiteVideoIn.release();
	waitKey(20);
	hiLiteVideoIn.open(fullName);
	if (!hiLiteVideoIn.isOpened()){
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return 0;
	}

// Open the for posting hiLites Video...the output of this program.
	fullName = dirPath + "\\forPosting\\forPost_" + fileName;
	hiLiteVideoOut.open(fullName, -1, hiLiteVideoIn.get(CV_CAP_PROP_FPS), Size(640, 360), true);
	if (!hiLiteVideoOut.isOpened()){
		cout << "ERROR Opening output file\n";
		getchar();
		return 0;
	}







// Now that setup is done, process selected hilites video.


	bool atVideoEnd = false;
	bool lastActionWasDelete = false;
	int vehiclesInFramesBuffer = 0;  // 
	string response;

	initializeVehicle(vehicle[0]);
	initializeVehicle(vehicle[1]);

	while (hiLiteVideoIn.get(CV_CAP_PROP_POS_FRAMES) < hiLiteVideoIn.get(CV_CAP_PROP_FRAME_COUNT)){

//		framePTR[vehIndex] = 0;
		if (vehiclesInFramesBuffer == 0 || (vehiclesInFramesBuffer == 1 && lastActionWasDelete)){
			bool atVehicleClipEnd = false;
			while (!atVehicleClipEnd){
				if (hiLiteVideoIn.get(CV_CAP_PROP_POS_FRAMES) <= hiLiteVideoIn.get(CV_CAP_PROP_FRAME_COUNT)){
					hiLiteVideoIn.read(vehicle[vehIndex].frames[vehicle[vehIndex].framePTR]);
					if (!blackFrame(vehicle[vehIndex].frames[vehicle[vehIndex].framePTR])){  //  If this is a black frame, keep it but don't show it
						if (vehicle[vehIndex].framePTR == 0){
							imshow("First Frame", vehicle[vehIndex].frames[0](Rect(0, top, 1280, 360)));
							waitKey(20);
						}
						imshow("Next Frame", vehicle[vehIndex].frames[vehicle[vehIndex].framePTR](Rect(0, top, 1280, 360)));
						waitKey(20);
						if (vehicle[vehIndex].markerFramePTR == 0 && upperWhiteSquare(vehicle[vehIndex].frames[vehicle[vehIndex].framePTR])) {
							vehicle[vehIndex].markerFramePTR = vehicle[vehIndex].framePTR;
							imshow("Marker Frame", vehicle[vehIndex].frames[vehicle[vehIndex].framePTR](Rect(0, top, 1280, 360)));
							waitKey(20);
						}
					}
					else { // Just read all black frame marking end of video for current vehicle
						atVehicleClipEnd = true;
// Get the stats associated with the vehicle just read in
						for (int i = 0; i < 7; i++){
							getline(hiLiteStats, vehicle[vehIndex].vehicleStats[i], ',');
						}
						getline(hiLiteStats, vehicle[vehIndex].vehicleStats[7]);

					}
					vehicle[vehIndex].framePTR++;
				}
				else{
					atVehicleClipEnd = true;
					atVideoEnd = true;
				}
			} // while reading frames for current vehicle
			vehiclesInFramesBuffer++;
		}


		if (!atVideoEnd){
			bool noAction = true;
			while (noAction){

				int vehicleSpeed = stoi(trim(vehicle[vehIndex].vehicleStats[7]));
				int vehicleArea = stoi(trim(vehicle[vehIndex].vehicleStats[6]));
				string recommendation;

				cout << endl << "Direction = " << vehicle[vehIndex].vehicleStats[3] << "  Area = " << vehicleArea
					<< " speed = " << vehicleSpeed << endl;
				

				if (vehicle[vehIndex].vehicleKind != UNK   ||  isVehicleKnown(vehicle[vehIndex])){
					cout << "Hey there's a match in the vehicle database!" << endl;
					recommendation = "Keep";
				}
				else {  // 	Decision logic for deciding whether to keep or delete for posting
					if (vehicleSpeed > 37 || (vehicleSpeed > 30 && vehicleArea > 80) || (vehicleArea > 150) )
						recommendation = "Keep";
					else recommendation = "Delete";
				}
				Mat newFrame;
				vehicle[vehIndex].frames[vehicle[vehIndex].framePTR - 2].copyTo(newFrame);


				if (trim(vehicle[vehIndex].vehicleStats[3]) == "L2R")
					putText(newFrame, recommendation, Point(950, top + 45), 2, 1, Scalar(CVRed), 2);
				else
					putText(newFrame, recommendation, Point(225, top + 45), 2, 1, Scalar(CVRed), 2);
				imshow("Next Frame", newFrame(Rect(0, top, 1280, 360)));
				waitKey(20);

				if (recommendation == "Delete"){
					cout << "Recommendation is to DELETE" << "  Do you agree?  (y/n/k) [y]: ";
					yesNo = "n";
					getline(cin, yesNo);
					if (yesNo == "k") response = "k";
					else if (yesNo == "n") response = "";
					else response = "d";
				}
				else {
					cout << "Recommendation is to KEEP" << "  Do you agree?  (y/n/d) [y]: ";
					yesNo = "n";
					getline(cin, yesNo);
					if (yesNo == "d") response = "d";
					else if (yesNo == "n") response = "";
					else response = "k";
				}


				if (response == ""){  // User has disagreed with recommendation.  Get what they want.
					if (lastActionWasDelete) cout << "(K)eep,  (D)elete,  (R)eplay,  (Q)uit,  (U)ndo   [d]: ";
					else cout << "(K)eep,  (D)elete,  (R)eplay,  (Q)uit   [d]: ";
					getline(cin, response);
				}


				if (response == "u" && lastActionWasDelete){  // "u"   undo delete performed on previoous step.
					lastActionWasDelete = false;
					vehIndex = 1 - vehIndex;  // Switch back to the buffer holding deleted vehicle frames;  give a second chance...
				//	replay(1, 250);
					replay(vehicle[vehIndex].framePTR - 1, 20);
					noAction = false;
				}


				else if (response == "r"){  // "s"  replay slow speed
					replay(vehicle[vehIndex].framePTR - 1, 150);
				}
				else if (response == "q"){  // "q"  quit.
					cout << "quitting" << endl;
					noAction = false;
				}
				else if (response == "k") {  // "k" keep vehicle in hiLites
					for (int i = 0; i < 15; i++)
						hiLiteVideoOut.write(vehicle[vehIndex].frames[0](Rect(leftSide, top, 640, 360)));
					for (int i = 0; i < vehicle[vehIndex].framePTR - 1; i++)
						hiLiteVideoOut.write(vehicle[vehIndex].frames[i](Rect(leftSide, top, 640, 360)));
					for (int i = 0; i < 20; i++)
						hiLiteVideoOut.write(vehicle[vehIndex].frames[vehicle[vehIndex].framePTR - 2](Rect(leftSide, top, 640, 360)));
					hiLiteVideoOut.write(vehicle[vehIndex].frames[vehicle[vehIndex].framePTR - 1](Rect(leftSide, top, 640, 360)));
					vehicle[vehIndex].framePTR = 0;
					noAction = false;

			// Does user want an image derived from this vehicle?		
					//cout << "Want an image made from middle frame?  (y/n) [n]: ";
					//yesNo = "n";
					//getline(cin, yesNo);
					//if (yesNo == "y"){
					//	cout << "Give user choice of which of three displayed frames to use.  Middle is default" << endl;
					//	cout << "Identify name to attach to file." << endl;
					//	cout << "Save file" << endl;
					//}
					//else cout << "You chose no." << endl;

					
					if (lastActionWasDelete){ // Two items in frames matrix; old one was deleted; current one kept.  Discard both now.
						vehiclesInFramesBuffer = 0;
						initializeVehicle(vehicle[0]);
						initializeVehicle(vehicle[1]);

					}
					else if (vehiclesInFramesBuffer == 1){ // lastActionWasDelete is  FALSE;  vehiclesInFramesBuffer == 1;  zero buffers.
						vehiclesInFramesBuffer = 0;
						initializeVehicle(vehicle[vehIndex]);
					}
					else {  // lastActionWasDelete is  FALSE;  vehiclesInFramesBuffer == 2;  It must be that older entry has just been kept.
						vehiclesInFramesBuffer = 1;
						initializeVehicle(vehicle[vehIndex]);  // zero out just kept vehicle.
						vehIndex = 1 - vehIndex; // switch pointer back to vehicle already in buffer
						//	replay(1, 250);  // Replay newer vehicle still in buffer.
						replay(vehicle[vehIndex].framePTR - 1, 20);
					}
					lastActionWasDelete = false;  // Last action was the keep, just done.
					cout << "Vehicle kept." << endl;
				}
				else /* if (response == "d") */  {  // "d"   delete up to and including last frame read.  Assume "d" if no other matches occur.
					if (lastActionWasDelete){ // handle two deletes in a row ==> get rid of older deleted vehicle frames.
					//	cout << "Second delete in a row.  Dropping older delete." << endl;
						vehiclesInFramesBuffer = 1;
						vehIndex = 1 - vehIndex;  // toggle input frame buffer pointer bwteen 0 and 1;
						initializeVehicle(vehicle[vehIndex]);  // This clears the next buffer to be read into, leaving the last buffer for an undo.
						noAction = false;
					}
					else{
						lastActionWasDelete = true;
					//	cout << "Previous action was not a delete, but this one is..  Toggling frame pointer." << endl;
						vehIndex = 1 - vehIndex;  // toggle input frame buffer pointer to vector for other vehicle frames;
						if (vehiclesInFramesBuffer < 2){
							initializeVehicle(vehicle[vehIndex]);  // This clears the next vehicle buffer to be read into if there isn't something already in it.
						}
						else replay(vehicle[vehIndex].framePTR - 1, 20); // Replay vehicle still in buffer
						noAction = false;
					}
					cout << "Vehicle deleted.  Undo is possible." << endl;
				}

			} // while no action
		}  // If not at video end

		if (response == "q")
			break;          //  Stop reading any frames and finish up.
	} // while frames remain to be read


	hiLiteVideoIn.release();
	hiLiteVideoOut.release();

	return 0;
}

