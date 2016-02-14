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

// ================
//Thank you Kyle Hounslow for your helpful Youtube videos on motion tracking in B/W
// ================

//***********************************************************************************************************************
//   This program processes video images of a typical two lane bidirectional street, tracks vehicles passing along that street,
// and forms an estimate of each vehicle's speed.  Speadsheet data, in the form of a csv file, is produced by this porgram.  Each
// entry in the csv file captures vehicle direction, frames in which the vehicle was analyzed for speed, the vehicle's profile area,
// and the vehicle's estimated speed. 
//   This program identifies vehicles in a video stream using what is classically known as "frame differencing", wherein
// two consecutive video frames are differenced, against a relatively static background, in order to detect where motion has
// occurred.  Often the shape arising in this differencing method approximates the shape of the vehicle.  More importantly, the
// leading and trailing edges of the vehicle can generally be discerned.  A sequence of frame differences can be used to approximate
// vehicle position over time, and therefore its velocity.
//   Frame differencing can be noisy.  Glare off of windshields, shadows, similarly colored objects in the background and other artifacts
// can make validity of a single frame difference questionable.  However, over time, most sequences of frame differences will produce
// sufficient useful information to allow for reliable tracking of motion.  As is done in most radar systems, tracking reliability can
// be enhanced significantly through the employment of a predictive tracking filter.  VST uses such a method to separately track
// both the front edge of a vehicle and its trailing edge, and to predict where useful data about the vehicle will appear in a
// subsequent frame difference.  The filtering/tracking method used in VST is a piecewise linear least squares method, which has
// proven reliable under many circumstances in many tracking applications.
//   VST is not perfect.  Mistracking can occur in the event frame difference data is too noisy to produce useful tracking, and therefore
// speed estimation information.  VST produces an optional highlights video, which is intended to be used as a quality checker.  Before
// making any results from VST public, the highlights video should be inspected manually for quality.
//  A feature of VST that I have not seen in similar tools in the public domain before is handling of bidirectional traffic.  VST predicts
// when two vehicles will pass and engages dead-reckoning to move the vehicle through conditions where raw data (frame difference images)
// yield little useful data (without significant image shape analysis, which itself can be easily fooled by noisy data).  Dead-reckoning
// of passing vehicles is working quite well in VST, based on my own observations of its ability to maintain track on the front bumper
// of vehicle as it passes another.  Your experience may vary depending on conditions I can't control.
//*****************************************************************************************************************************


#include <opencv\cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "GLobals.h""
#include <iostream>
#include <fstream>
#include <queue>
#include "VehicleDynamics.h"
#include "Projection.h"
#include "Snapshot.h"



using namespace std;
using namespace cv;

const int MAX_NUM_OBJECTS = 30; // Max number of objects allowed to be retunred by contours
const int MIN_OBJECT_AREA = 30 * 35;  // Very sensitive to pedestrians, bicyclists and other small things.
int numObjects = 0;  // 
Rect coalescedRectangle;  //  The collection of blobs that represent a vehicles projected area.

vector<VehicleDynamics> vehiclesGoingRight;
vector<VehicleDynamics> vehiclesGoingLeft;

bool bailing = false;



// ........................................................ Globals shared between setup() and main() ................................................
ofstream traceFile;
ofstream statsFile;
ifstream directoryList;
ifstream filesList;
VideoCapture capture;  //video capture object.
bool moreFilesToDo = true;  //  Used to control file processing loop in main.
string yesNoAll = "n";  // Indicates whetehr one file (Y) or multiple (*) are to be processed.
string fileName;  // Name of avi file currently being processed.
string dirPath; // path to fileName
string fileMid; // The date part of the file name placed there by the Foscam camera
int objDelay = 1250;  // delay to be used when objects are detected in ROI;  Can be changed through use of "f" and "s" keys while running
bool pleaseTrace = false;  // If you want a trace file (lots of debug info)
bool highLightsPlease = false;
int speedLimit = 25;  // User supplied speed limit, used for color choice when posting speed
int egregiousSpeedLowerBound = 35;    // User supplied egregious speed lower bound, used for color choice when posting speed
int crazySpeed = 55;
int highLightsSpeedLower = 35; // Default lower threshold for including vehicles in the highlights file
int highLightsSpeedUpper = 100; // Default upper threshold for including vehicles in the highlights file
int minimumProfileArea = 100;  // Default lower bound on size of large vehicle to be added to highlights if speeding over speed limit.
int frameNumber; // Current framenumber being processed, relative to beginning of file "fileName"
double startFrame = 0.0;
Rect AnalysisBox;  // the coordinates and extents of the region beng analyzed for vehicle motion.  Subregion of frames read in.
VideoWriter hiLiteVideo; // For writing highlights...the Scofflaws
Globals g;

Mat frame1, frame2; // Frames read by main, to use in frame differencing, and for display.
//......................................................................................................................................................

//int to string helper function
string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

// Interact with the user via command line to gather setup information.
// Also, call readconfig() to read in (from file VST.cfg) and assign configuration data.
void setup(){

// Get configuration values from VST.cfg and set corresponding objects in Globals.h to read-in values.
	if (!g.readConfig()){
		cout << "Reading of config file appears to have failed.   Exiting" << endl;
		exit(-1);
	}
// PixelLeft is always zero relative to AnalysisBoxLeft;  PixelRight depends on AnalysisBoxWidth/
	g.pixelRight = g.AnalysisBoxWidth; // index of rightmost pixel in AnalysisBox.
	AnalysisBox = Rect(g.AnalysisBoxLeft, g.AnalysisBoxTop, g.AnalysisBoxWidth, g.AnalysisBoxHeight);  // For use when performing speed analysis in cropped region

	Mat frame;
	string dirName;
	string camPath = g.dataPathPrefix + "\\IPCam\\";

	cout << endl << endl;

// Get directory containing file(s) to be processed
	string toSysString = "dir " + camPath + " /b > " + camPath + "directories.txt";
	const char * toSysStringC = toSysString.c_str();
//	system("dir g:\\LocustData\\IPCam /b > g:\\LocustData\\IPCam\\directories.txt");
	system(toSysStringC);
	string yesNo = "n";
	while (yesNo == "n"){
		directoryList.open(camPath + "directories.txt");
		while (getline(directoryList, dirName)){
			cout << "Want the directory " << dirName.substr(0, 4) + " " + dirName.substr(4, 2) + " " + dirName.substr(6, 2) << "  (y/n) [n]: ";
			getline(cin, yesNo);
			if (yesNo == "y") break;
		}
		directoryList.close();
	}

// Go through the file names in the selected directory for the user.
	dirPath = camPath + dirName;
	string sysString = "dir " + dirPath + "\\*.avi /b > " + dirPath + "\\files.txt";
	const char * c = sysString.c_str();
	system(c); // copy the file names from the chosen directory to file "files.txt" in the same directory.

	// Get file user wants
	while (yesNoAll == "n"){
		filesList.open(dirPath + "\\files.txt");
		while (getline(filesList, fileName)){
			cout << "Want the file " << fileName.substr(0, 15) + "_" + fileName.substr(15, 10) << "  (y/n/*) [n]: ";
			getline(cin, yesNoAll);
			if (yesNoAll == "y" || yesNoAll == "*") break;  // User has chosen one file, or all in directory
		}
		filesList.close();
	}

	// Do a one-time setup of region of interest, obstructions and speed posts
	string FullName = dirPath + "\\" + fileName;
	capture.open(FullName);
	if (!capture.isOpened()){
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return;
	}
	capture.read(frame);
	cv::line(frame, Point(g.AnalysisBoxLeft, g.AnalysisBoxTop), Point(g.AnalysisBoxLeft + g.AnalysisBoxWidth, g.AnalysisBoxTop), Scalar(CVYellow), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft, g.AnalysisBoxTop + g.AnalysisBoxHeight), Point(g.AnalysisBoxLeft + g.AnalysisBoxWidth, g.AnalysisBoxTop + g.AnalysisBoxHeight), Scalar(CVYellow), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft, g.AnalysisBoxTop), Point(g.AnalysisBoxLeft, g.AnalysisBoxTop + g.AnalysisBoxHeight), Scalar(CVYellow), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + g.AnalysisBoxWidth, g.AnalysisBoxTop), Point(g.AnalysisBoxLeft + g.AnalysisBoxWidth, g.AnalysisBoxTop + g.AnalysisBoxHeight), Scalar(CVYellow), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + g.speedLineLeft, 85 + g.AnalysisBoxTop), Point(g.AnalysisBoxLeft + g.speedLineLeft, 160 + g.AnalysisBoxTop), Scalar(CVWhite), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + g.speedLineRight, 85 + g.AnalysisBoxTop), Point(g.AnalysisBoxLeft + g.speedLineRight, 160 + g.AnalysisBoxTop), Scalar(CVWhite), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + g.obstruction[0], 85 + g.AnalysisBoxTop), Point(g.AnalysisBoxLeft + g.obstruction[0], 160 + g.AnalysisBoxTop), Scalar(CVYellow), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + g.obstruction[1], 85 + g.AnalysisBoxTop), Point(g.AnalysisBoxLeft + g.obstruction[1], 160 + g.AnalysisBoxTop), Scalar(CVYellow), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + 10, g.AnalysisBoxTop + g.R2LStreetY), Point(g.AnalysisBoxLeft + g.AnalysisBoxWidth - 20, g.AnalysisBoxTop + g.R2LStreetY), Scalar(CVOrange), 2);
	cv::line(frame, Point(g.AnalysisBoxLeft + 10, g.AnalysisBoxTop + g.L2RStreetY), Point(g.AnalysisBoxLeft + g.AnalysisBoxWidth - 20, g.AnalysisBoxTop + g.L2RStreetY), Scalar(CVPurple), 2);

	switch (waitKey(20)){};
	cv::imshow("Full Frame", frame);
	switch (waitKey(20)){};

	cout << "Are Analysis Box, Speed Measuring Zone, " << endl << "    Obstruction Framing, and Hubcap Lines OK (y|n) [y] ?  ";
	getline(cin, yesNo);
	if (!yesNo.empty() & (yesNo == "n")){ 
		cout << "You'll need to change values in VST.cfg.  Terminating.   Hit enter to exit program." << endl;
		getline(cin, yesNo);
		cv::destroyWindow("Full Frame");
		capture.release();
		exit(-1);
	}
	else{
		if (yesNo.substr(0, 1) == "y")
			cout << "Glad you're happy." << endl;;
	}
	cv::destroyWindow("Full Frame");

	filesList.open(dirPath + "\\files.txt"); // Done for main() to access files contained therein

// Want a trace file?
	pleaseTrace = false;
	cout << endl << "Want a trace file (y/n) [n]? : ";
	getline(cin, yesNo);
	if (!yesNo.empty()) pleaseTrace = (yesNo == "y");

// Open trace file (if requested) and stats file
	if (yesNoAll == "*"){ // give trace and stats files names based on directory name
		if (pleaseTrace) traceFile.open(g.dataPathPrefix + "\\trace\\trace_" + dirName + ".txt");
		statsFile.open(g.dataPathPrefix + "\\stats\\stats_" + dirName + ".csv");
	}
	else{ // yesNoAll == "y" which means only one file to process; give it name corresponding to input file name
		fileMid = fileName.substr(7, 14);
		if (pleaseTrace) traceFile.open(g.dataPathPrefix + "\\trace\\trace_" + fileMid.substr(0, 8) + "_" + fileMid.substr(8, 6) + ".txt");
		statsFile.open(g.dataPathPrefix + "\\stats\\stats_" + fileMid.substr(0, 8) + "_" + fileMid.substr(8, 6) + ".csv");
	}

	statsFile << ", , Frame, Direction, StartFrame, EndFrame, # Frames, StartPix, EndPix, DeltaPix, VehicleArea, , estSpeed" << endl;

	string answer;
	cout << endl << "Speed Limit: (int) [" + intToString(speedLimit) + "]: ";
	getline(cin, answer);
	if (!answer.empty()) speedLimit = stoi(answer);
	cout << endl;

	egregiousSpeedLowerBound = speedLimit + 10;
	cout << "Egregious Speed Lower Bound: (int) [" + intToString(egregiousSpeedLowerBound) + "]: ";
	getline(cin, answer);
	if (!answer.empty()) egregiousSpeedLowerBound = max(stoi(answer), speedLimit);
	cout << endl;

	crazySpeed = egregiousSpeedLowerBound + 20;  // Stats reporting will flag anything faster than this.

// What frame number would you like to start with in the first file?

	startFrame = 0.0;
	cout << "Frame number to start with in first file (int) [0]? : ";
	getline(cin, answer);
	if (!answer.empty()) startFrame = stod(answer);

// Want a highlights file?
	highLightsPlease = false;
	cout << endl << "Want a highlights file (y/n) [n]? : ";
	getline(cin, yesNo);
	if (!yesNo.empty()) highLightsPlease = (yesNo == "y");

// What lower threshold speed for being added to highlights?
	if (highLightsPlease){
		cout << endl << "Threshold lower speed for highlights file: (int) [" + intToString(highLightsSpeedLower) + "]: ";
		getline(cin, answer);
		if (!answer.empty()) highLightsSpeedLower = stoi(answer);
// What upper threshold speed for being added to highlights?
		cout << endl << "Threshold upper bound on speed for highlights file: (int) [" + intToString(highLightsSpeedUpper) + "]: ";
		getline(cin, answer);
		if (!answer.empty()) highLightsSpeedUpper = stoi(answer);
// What minimum profile area should be used for adding speeding vehicles to highlights?
		cout << endl << "Min area of large speeding vehicle to be added to highlights (int) [" + intToString(minimumProfileArea) + "]: ";
		getline(cin, answer);
		if (!answer.empty()) minimumProfileArea = stoi(answer);
		cout << endl;
		if (yesNoAll == "*"){ // give trace and stats files names based on directory name
			hiLiteVideo.open(g.dataPathPrefix + "\\HiLites\\Hilites_" + dirName + ".avi",
//				CV_FOURCC('X', '2', '6', '4'), capture.get(CV_CAP_PROP_FPS), Size(1280, 720), true);
			-1, capture.get(CV_CAP_PROP_FPS), Size(1280, 720), true); // bug in OpenCV open function.  x264 has to be picked from list.  Argh.
		}
		else{ // yesNoAll == "y" which means only one file to process; give it name corresponding to input file name
			fileMid = fileName.substr(7, 14);
			hiLiteVideo.open(g.dataPathPrefix + "\\HiLites\\Hilites_" + fileMid.substr(0, 8) + "_" + fileMid.substr(8, 6) + ".avi",
//				CV_FOURCC('X', '2', '6', '4'), capture.get(CV_CAP_PROP_FPS), Size(1280, 720), true);
			-1, capture.get(CV_CAP_PROP_FPS), Size(1280, 720), true); // bug in OpenCV open function.  x264 has to be picked from list.  Argh.
			
		}
		if (!hiLiteVideo.isOpened()){
			cout << "ERROR Opening HiLites File\n";
			getchar();
			return;
		}
	}
	capture.release();
	return;
}



string vStateString(vehicleStatus inState){
	// {entering, inMiddle, exiting, exited};
	if (inState == entering) return "entering";
	else if (inState == inMiddle) return "inMiddle";
	else if (inState == exiting) return "exiting";
	else return "exited";
}


string statusString(statusTypes inStatus){
	// statusTypes {ImOK, deleteWithStats, lostTrack, negVelocity }
	if (inStatus == ImOK) return "ImOK";
	else if (inStatus == deleteWithStats) return "deleteWithStats";
	else if (inStatus == lostTrack) return "lostTrack";
	else return "negVelocity";

}


string overlapString(OverlapType inOverlap){
	//	none, rearOnly, frontOnly, bothOverlap
	if (inOverlap == none) return "none";
	else if (inOverlap == rearOnly) return "rearOnly";
	else if (inOverlap == frontOnly) return "frontOnly";
	else return "bothOverlap";
}



void hesitate(int code){  // easy breakpoint for debugging when you don't want to fire up a debugger.
	cout << frameNumber << "  Program paused, input value is: " << code << "   Press 'p' to resume" << endl;
	while (waitKey() != 112);
}



Rect coalesce(Rect rectangles[], int numRects, int loX, int hiX, grabType how){
	// For a specified region of interest, put a single rectangle around all of the external contours the contours funtion found.
	Rect retRect;
	int firstOK = -1;
	for (int i = 0; i < numRects; i++){ // find first if any rectangle that is in spec'd bounds
		if (rectangles[i].x <= hiX && (rectangles[i].x + rectangles[i].width) >= loX){ // Any overlap at all?
			retRect = rectangles[i];
			if (how == strict) {  // Keep it inside LoX...HiX
				retRect.x = max(rectangles[i].x, loX);
				retRect.width = min(rectangles[i].x + rectangles[i].width, (loX + hiX)) - retRect.x;
			}
			firstOK = i;
			break;
		}
	}
	if (firstOK == -1){
		if(pleaseTrace) traceFile << "<" << frameNumber << "> Coalesce finds no acceptable objects between loX:  " << loX << " and  hiX:  " << hiX << endl;
		return Rect{ -1, 0, 0, 0 };
	}
	for (int i = (firstOK + 1); i < numRects; i++){
		if (rectangles[i].x <= hiX && (rectangles[i].x + rectangles[i].width) >= loX){    // overlap with requested bounded area
			int leftMore = min(retRect.x, rectangles[i].x);
			int rightMore = max(retRect.x + retRect.width, rectangles[i].x + rectangles[i].width);
			if (how == strict) {  // Keep it inside LoX...HiX
				leftMore = max(leftMore, loX);
				rightMore = min(rightMore, (loX + hiX));
			}
			retRect.x = leftMore;
			retRect.width = rightMore - retRect.x;

			int topMore = min(retRect.y, rectangles[i].y);
			int bottomMore = max(retRect.y + retRect.height, rectangles[i].y + rectangles[i].height);
			retRect.y = topMore;
			retRect.height = bottomMore - retRect.y;
		}
	}
	if (pleaseTrace) traceFile << "<" << frameNumber << "> Coalesce finds object in [" << loX << ", " << hiX << "] --> Rect:   " << retRect.x << ", "
		<< retRect.y << ", " << retRect.width << ",   " << retRect.height;// << endl;
	return retRect;

}

bool meetsHLRCriterion(int inSpeed, int inArea){ // Does the vehicle speed meet criterion for HiLites reel?
	return highLightsPlease 
		&& (   ((inSpeed >= highLightsSpeedLower) && (inSpeed <= highLightsSpeedUpper))
		||    /* ((inSpeed >= (highLightsSpeedLower - 8)) && */ (inArea >= g.largeVehicleArea) /*)*/);
}

void displayAnalysisGoingRight(int inFrameNum, int index, Rect rectangle, OverlapType Olap, Mat &AnalysisFrame, int estSpeed){
// Display the green rectangle with the leading blue vertical line (hopefully on the front bumper) representng the *predicted*
// area occupied by the vehicle.  Also display the velocity of the vehicle after it has has passed its second white post delineating the end
// of the L2R speed measuring zone.  Also, save frame until it's known whether this vehicle will be added to highlights video.
	int x = rectangle.x;
	int y = rectangle.y;
	int wd = rectangle.width;
	int ht = rectangle.height;
	cv::line(AnalysisFrame, Point(x, y), Point(x + wd, y), Scalar(CVGreen), 2);
	cv::line(AnalysisFrame, Point(x, y + ht), Point(x + wd, y + ht), Scalar(CVGreen), 2);
	if (Olap == rearOnly || Olap == bothOverlap)
		cv::line(AnalysisFrame, Point(x, y), Point(x, y + ht), Scalar(CVRed), 2);
	else
		cv::line(AnalysisFrame, Point(x, y), Point(x, y + ht), Scalar(CVGreen), 2);
	if (Olap == frontOnly || Olap == bothOverlap)
		cv::line(AnalysisFrame, Point(x + wd, y), Point(x + wd, y + ht), Scalar(CVRed), 2);
	else
		cv::line(AnalysisFrame, Point(x + wd, y), Point(x + wd, y + ht), Scalar(CVBlue), 2);
	if (estSpeed > 0)  
		if (estSpeed <= speedLimit)  
			putText(AnalysisFrame, intToString(estSpeed) + " MPH", Point(g.pixelRight-180, 30), 2, 1, Scalar(CVGreen), 2);
		else if (estSpeed < egregiousSpeedLowerBound) 
			putText(AnalysisFrame, intToString(estSpeed) + " MPH", Point(g.pixelRight-180, 30), 2, 1, Scalar(CVYellow), 2);
		else                         
			putText(AnalysisFrame, intToString(estSpeed) + " MPH", Point(g.pixelRight-180, 30), 2, 1, Scalar(CVRed), 2);
	if (highLightsPlease){
		if (vehiclesGoingRight[index].getTrackStartPixel() == 0)
			vehiclesGoingRight[index].holdFrame(AnalysisFrame); // if vehicle hasnt passed start post yet, keep last frame in case about to cross into speed zone.
		else {  // past the start post,
			if (vehiclesGoingRight[index].getNumberSavedFrames() == 0) // First time to save a frame for hilites reel?
				vehiclesGoingRight[index].saveFrame(vehiclesGoingRight[index].getHeldFrame()); // then start up saving frames to possibly copy to hilites file later on.
			if (estSpeed <= 0) vehiclesGoingRight[index].saveFrame(AnalysisFrame); // Be saving frames past the start post.
			else  // estSpeed is > 0 meaning vehicle has passed end post
				if (meetsHLRCriterion(estSpeed, vehiclesGoingRight[index].getArea()) && vehiclesGoingRight[index].getTrackEndFrame() == frameNumber) // No use saving frames if vehicle doesn't meet hilites Reel criterion
					vehiclesGoingRight[index].saveFrame(AnalysisFrame);
		}
	}
	if (pleaseTrace) traceFile << "<" << frameNumber << "> DisplayGoingRight... Rect:  " << x << ", " << y << ", " << wd << ",  " << ht
		<<endl << endl;
}


void displayAnalysisGoingLeft(int inFrameNum, int index, Rect rectangle, OverlapType Olap, Mat &AnalysisFrame, int estSpeed){
// Display the green rectangle with the leading blue vertical line (hopefully on the front bumper) representng the *predicted*
// area occupied by the vehicle.  Also display the velocity of the vehicle after it has has passed its second white post, delineating the end
// of the R2L speed measuring zone.  Also, save frame until it's known whether this vehicle will be added to highlights video.
	int x = rectangle.x;
	int y = rectangle.y;
	int wd = rectangle.width;
	int ht = rectangle.height;
	cv::line(AnalysisFrame, Point(x, y), Point(x + wd, y), Scalar(CVGreen), 2);
	cv::line(AnalysisFrame, Point(x, y + ht), Point(x + wd, y + ht), Scalar(CVGreen), 2);
	if (Olap == frontOnly || Olap == bothOverlap)
		cv::line(AnalysisFrame, Point(x, y), Point(x, y + ht), Scalar(CVRed), 2);
	else
		cv::line(AnalysisFrame, Point(x, y), Point(x, y + ht), Scalar(CVBlue), 2);
	if (Olap == rearOnly || Olap == bothOverlap)
		cv::line(AnalysisFrame, Point(x + wd, y), Point(x + wd, y + ht), Scalar(CVRed), 2);
	else
		cv::line(AnalysisFrame, Point(x + wd, y), Point(x + wd, y + ht), Scalar(CVGreen), 2);
	if (estSpeed > 0)
		if(estSpeed <= speedLimit) 
			putText(AnalysisFrame, intToString(estSpeed) + " MPH", Point(g.pixelLeft, 30), 2, 1, Scalar(CVGreen), 2);
		else if (estSpeed < egregiousSpeedLowerBound)
			putText(AnalysisFrame, intToString(estSpeed) + " MPH", Point(g.pixelLeft, 30), 2, 1, Scalar(CVYellow), 2);
		else 
			putText(AnalysisFrame, intToString(estSpeed) + " MPH", Point(g.pixelLeft, 30), 2, 1, Scalar(CVRed), 2);
	if (highLightsPlease){
		if (vehiclesGoingLeft[index].getTrackStartPixel() == 0) 
			vehiclesGoingLeft[index].holdFrame(AnalysisFrame); // if vehicle hasnt passed start post yet, keep last frame in case about to cross into speed zone.
		else {  // past the start post,
			if (vehiclesGoingLeft[index].getNumberSavedFrames() == 0) // First time to save a frame for hilites reel?
				vehiclesGoingLeft[index].saveFrame(vehiclesGoingLeft[index].getHeldFrame()); // then start up saving frames to possibly copy to hilites file later on.
			if (estSpeed <= 0) vehiclesGoingLeft[index].saveFrame(AnalysisFrame); // Be saving frames past the start post.
			else  // estSpeed is > 0 meaning vehicle has passed end post
				if (meetsHLRCriterion(estSpeed, vehiclesGoingLeft[index].getArea()) && vehiclesGoingLeft[index].getTrackEndFrame() == frameNumber) // No use saving frames after end post if vehicle doesn't meet hilites Reel criterion
					vehiclesGoingLeft[index].saveFrame(AnalysisFrame);
		}
	}
	if (pleaseTrace) traceFile << "<" << frameNumber << "> DisplayGoingLeft... Rect:  " << x << ", " << y << ", " << wd << ",  " << ht
		<< endl << endl;
}







void logL2Rstats(bool isOK, int index){
// Final entries for L2R vehicle just completing speed analysis are placed in trace file and in stats files.  Video output to highlights
//	file for qualifying vehicles is performed.
	int frames = max(vehiclesGoingRight[index].getTrackEndFrame() - vehiclesGoingRight[index].getTrackStartFrame(), 1);
	int estSpeed = vehiclesGoingRight[index].getFinalSpeed();
	if (pleaseTrace) traceFile << "<" << frameNumber << ">   Entry frame: " << vehiclesGoingRight[index].getTrackStartFrame()
		<< "   Exit frame : " << vehiclesGoingRight[index].getTrackEndFrame()
		<< "   # frames: " << (vehiclesGoingRight[index].getTrackEndFrame() - vehiclesGoingRight[index].getTrackStartFrame())
		<< endl
		<< "         Entry pixel: " << vehiclesGoingRight[index].getTrackStartPixel()
		<< "   Exit pixel: " << vehiclesGoingRight[index].getTrackEndPixel()
		<< "   # Pixels: " << vehiclesGoingRight[index].getTrackEndPixel() - vehiclesGoingRight[index].getTrackStartPixel()
		<< "             Est speed: " << estSpeed
		<< endl << "> > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > >"
		<< endl << endl << endl;
	if ((estSpeed >= 18.0) && isOK){
		statsFile << fileName.substr(7, 8) << ", " << fileName.substr(15, 6) << ", "
			<< frameNumber << ", " << g.L2RDirection << ", " << vehiclesGoingRight[index].getTrackStartFrame() << ", "
			<< vehiclesGoingRight[index].getTrackEndFrame() << ", "
			<< frames << ", "
			<< vehiclesGoingRight[index].getTrackStartPixel() << ", "
			<< vehiclesGoingRight[index].getTrackEndPixel() << ", "
			<< vehiclesGoingRight[index].getTrackEndPixel() - vehiclesGoingRight[index].getTrackStartPixel() << ", "
			<< vehiclesGoingRight[index].getArea() << ", , "
			<< estSpeed;
		if (estSpeed < 0 || estSpeed > crazySpeed) statsFile << ", *****";
		statsFile << endl;
		if (meetsHLRCriterion(estSpeed, vehiclesGoingRight[index].getArea())){
			Mat zero = Mat::zeros(Size(1280, 720), frame1.type());
			int speedleft = AnalysisBox.x + g.speedLineLeft; // Left boundary may have moved right for ROI boundary.
			int speedRight = AnalysisBox.x + g.speedLineRight;
			int arrowY = AnalysisBox.y - 32;
			frame1(Rect(0, 0, 240, 29)).copyTo(zero(Rect(500, 440, 240, 29))); // Get date/time from input frame and copy it to just below the ROI.
			arrowedLine(zero, Point(speedleft + 10, arrowY), Point(speedleft + 60, arrowY), Scalar(CVPurple), 5);
			for (int i = 0; i < 5; i++){
				vehiclesGoingRight[index].getSavedFrame(0).copyTo(zero(AnalysisBox));
				hiLiteVideo.write(zero);
				waitKey(10);
			}
			arrowedLine(zero, Point(speedleft + 10, arrowY), Point(speedleft + 60, arrowY), Scalar(CVBlack), 5);
			int midPoint = (speedleft + speedRight) / 2;
			arrowedLine(zero, Point(midPoint - 25, arrowY), Point(midPoint + 25, arrowY), Scalar(CVPurple), 5);
			for (int i = 0; i <= vehiclesGoingRight[index].getNumberSavedFrames() - 1; i++){
				vehiclesGoingRight[index].getSavedFrame(i).copyTo(zero(AnalysisBox));
				if (i == vehiclesGoingRight[index].getNumberSavedFrames() - 1){
					arrowedLine(zero, Point(midPoint - 25, arrowY), Point(midPoint + 25, arrowY), Scalar(CVBlack), 5);
					arrowedLine(zero, Point(speedRight - 60, arrowY), Point(speedRight - 10, arrowY), Scalar(CVPurple), 5);
					if (estSpeed >= egregiousSpeedLowerBound) 
						putText(zero, intToString(estSpeed) + " MPH", Point(speedRight - 125, AnalysisBox.y - 55), 2, 1, Scalar(CVRed), 2);
					else if (estSpeed > speedLimit)
						putText(zero, intToString(estSpeed) + " MPH", Point(speedRight - 125, AnalysisBox.y - 55), 2, 1, Scalar(CVYellow), 2);
					else putText(zero, intToString(estSpeed) + " MPH", Point(speedRight - 125, AnalysisBox.y - 55), 2, 1, Scalar(CVGreen), 2);
				}
				hiLiteVideo.write(zero);
				waitKey(10);
			}
			for (int i = 0; i < 5; i++){
				vehiclesGoingRight[index].getSavedFrame(vehiclesGoingRight[index].getNumberSavedFrames() - 1).copyTo(zero(AnalysisBox));
				hiLiteVideo.write(zero);
				waitKey(10);
			}
			hiLiteVideo.write(Mat::zeros(Size(1280, 720), frame1.type())); //  write a partition between vehicles for HiLites processor to detect.
		}
	}
}

void logR2Lstats(bool isOK, int index){
// Final entries for R2L vehicle just completing speed analysis are placed in trace file and in stats files.  Video output to highlights
//	file for qualifying vehicles is performed.
	int frames = max(vehiclesGoingLeft[index].getTrackEndFrame() - vehiclesGoingLeft[index].getTrackStartFrame(), 1);
	int estSpeed = vehiclesGoingLeft[index].getFinalSpeed();
	if (pleaseTrace) traceFile
		<< "<" << frameNumber << ">   Start frame: " << vehiclesGoingLeft[index].getTrackStartFrame()
		<< "   End frame : " << vehiclesGoingLeft[index].getTrackEndFrame()
		<< "   # frames: " << (vehiclesGoingLeft[index].getTrackEndFrame() - vehiclesGoingLeft[index].getTrackStartFrame())
		<< endl
		<< "          Start pixel: " << vehiclesGoingLeft[index].getTrackStartPixel()
		<< "   End pixel: " << vehiclesGoingLeft[index].getTrackEndPixel()
		<< "   # Pixels: " << vehiclesGoingLeft[index].getTrackStartPixel() - vehiclesGoingLeft[index].getTrackEndPixel()
		<< "             Est speed: " << estSpeed
		<< endl << "< < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < <"
		<< endl << endl << endl;
	if ((estSpeed >= 18.0) && isOK){
		statsFile << fileName.substr(7, 8) << ", " << fileName.substr(15, 6) << ", "
			<< frameNumber << ", " << g.R2LDirection << ", " << vehiclesGoingLeft[index].getTrackStartFrame() << ", "
			<< vehiclesGoingLeft[index].getTrackEndFrame() << ", "
			<< frames << ", "
			<< vehiclesGoingLeft[index].getTrackStartPixel() << ", "
			<< vehiclesGoingLeft[index].getTrackEndPixel() << ", "
			<< vehiclesGoingLeft[index].getTrackStartPixel() - vehiclesGoingLeft[index].getTrackEndPixel() << ", "
			<< vehiclesGoingLeft[index].getArea() << ", , "
			<< estSpeed;
		if (estSpeed < 0 || estSpeed > crazySpeed) statsFile << ", *****";
		statsFile << endl;
		if (meetsHLRCriterion(estSpeed, vehiclesGoingLeft[index].getArea())){
			Mat zero = Mat::zeros(Size(1280, 720), frame1.type());
			int speedleft = AnalysisBox.x + g.speedLineLeft;
			int speedRight = AnalysisBox.x + g.speedLineRight;
			int arrowY = AnalysisBox.y - 32;
			frame1(Rect(0, 0, 240, 29)).copyTo(zero(Rect(500, 440, 240, 29))); // Get date/time from input frame and copy it to just below the ROI.
			arrowedLine(zero, Point(speedRight - 10, arrowY), Point(speedRight - 60, arrowY), Scalar(CVOrange), 5);
			for (int i = 0; i < 5; i++){
				vehiclesGoingLeft[index].getSavedFrame(0).copyTo(zero(AnalysisBox));
				hiLiteVideo.write(zero);
				waitKey(10);
			}
			arrowedLine(zero, Point(speedRight - 10, arrowY), Point(speedRight - 60, arrowY), Scalar(CVBlack), 5);
			int midPoint = (speedleft + speedRight) / 2;
			arrowedLine(zero, Point(midPoint + 25, arrowY), Point(midPoint - 25, arrowY), Scalar(CVOrange), 5);
			for (int i = 0; i <= vehiclesGoingLeft[index].getNumberSavedFrames() - 1; i++){
				vehiclesGoingLeft[index].getSavedFrame(i).copyTo(zero(AnalysisBox));
				if (i == vehiclesGoingLeft[index].getNumberSavedFrames() - 1){
					arrowedLine(zero, Point(midPoint + 25, arrowY), Point(midPoint - 25, arrowY), Scalar(CVBlack), 5);
					arrowedLine(zero, Point(speedleft + 60, arrowY), Point(speedleft + 10, arrowY), Scalar(CVOrange), 5);
					if (estSpeed >= egregiousSpeedLowerBound)
						putText(zero, intToString(estSpeed) + " MPH", Point(speedleft, AnalysisBox.y - 55), 2, 1, Scalar(CVRed), 2);
					else if (estSpeed > speedLimit)
						putText(zero, intToString(estSpeed) + " MPH", Point(speedleft, AnalysisBox.y - 55), 2, 1, Scalar(CVYellow), 2);
					else putText(zero, intToString(estSpeed) + " MPH", Point(speedleft, AnalysisBox.y - 55), 2, 1, Scalar(CVGreen), 2);
				}
				hiLiteVideo.write(zero);
				waitKey(10);
			}
			for (int i = 0; i < 5; i++){
				vehiclesGoingLeft[index].getSavedFrame(vehiclesGoingLeft[index].getNumberSavedFrames() - 1).copyTo(zero(AnalysisBox));
				hiLiteVideo.write(zero);
				waitKey(10);
			}
			hiLiteVideo.write(Mat::zeros(Size(1280, 720), frame1.type())); //  write a partition between vehicles for HiLites processor to detect.
		}
	}
}


OverlapType doesL2ROverlapAnyR2L(int L2RIndex, vector<Projection> vehiclesL2R, vector<Projection> vehiclesR2L, int projectedR2LSize){
// Function name tells it all.  Does a selected L2R vehicle overlap any R2L vehicles?  Indicate which bumpers overlap.
	bool frontOverlap = false;
	bool rearOverlap = false;
	int L2RRearBumper = max(vehiclesL2R[L2RIndex].getBox().x - (4 * g.SLOP), g.pixelLeft);
	int L2RFrontBumper = min(L2RRearBumper + vehiclesL2R[L2RIndex].getBox().width + (5 * g.SLOP), g.pixelRight);  // 5x makes up for SLOP subtracted from rear bumper.
	for (int R2LIndex = 0; R2LIndex < projectedR2LSize; R2LIndex++){
		int R2LFrontBumper = vehiclesR2L[R2LIndex].getBox().x;
		int R2LRearBumper = R2LFrontBumper + vehiclesR2L[R2LIndex].getBox().width;
		if ((L2RFrontBumper >= R2LFrontBumper) && (L2RFrontBumper <= R2LRearBumper)) frontOverlap = true;
		if ( ( (L2RRearBumper >= R2LFrontBumper) && (L2RRearBumper <= R2LRearBumper) )
		/*	|| ((L2RRearBumper <= R2LFrontBumper) && (L2RFrontBumper >= R2LRearBumper)) */ ) rearOverlap = true;  // Handle full obstruction case
	}
	// none, rearOnly, frontOnly, bothOverlap
	if (frontOverlap && rearOverlap) return bothOverlap;
	if (frontOverlap) return frontOnly;
	if (rearOverlap) return rearOnly;
	return none;
}


OverlapType doesR2LOverlapAnyL2R(int R2LIndex, vector<Projection> vehiclesR2L, vector<Projection> vehiclesL2R, int projectedL2RSize){
// Function name tells it all.  Does a selected R2L vehicle overlap any L2R vehicles?  Indicate which bumpers overlap.
	bool frontOverlap = false;
	bool rearOverlap = false;
	int R2LFrontBumper = max(vehiclesR2L[R2LIndex].getBox().x - g.SLOP, g.pixelLeft);
	int R2LRearBumper = min(R2LFrontBumper + vehiclesR2L[R2LIndex].getBox().width + (5 * g.SLOP), g.pixelRight);  // 4x makes up for SLOP subtracted from front bumper.
	for (int L2RIndex = 0; L2RIndex < projectedL2RSize; L2RIndex++){
		int L2RRearBumper = vehiclesL2R[L2RIndex].getBox().x;
		int L2RFrontBumper = L2RRearBumper + vehiclesL2R[L2RIndex].getBox().width;
		if ( (R2LFrontBumper <= L2RFrontBumper) && (R2LFrontBumper >= L2RRearBumper)) frontOverlap = true;
		if ( ( (R2LRearBumper <= L2RFrontBumper) && (R2LRearBumper >= L2RRearBumper) ) 
		/*	|| ((R2LRearBumper >= L2RFrontBumper) && (R2LFrontBumper <= L2RRearBumper)) */  ) rearOverlap = true;   // Handle full obstruction case
	}
	// none, rearOnly, frontOnly, bothOverlap
	if (frontOverlap && rearOverlap) return bothOverlap;
	if (frontOverlap) return frontOnly;
	if (rearOverlap) return rearOnly;
	return none;
}



//  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 
// Given next differential image, use projections of all known in-track vehicles, as well as information about newly entering vehicles, to identify and process
// all that are 1) exiting, deleted, entering, overtaking, occluding, occluded, or simply moving forward.  If objects are detected in the region of interest, they
// are bracketed and associated with entering or already known vehicles, and this new information is preserved for each vehicle in a call to addSnap(), one call for
// each vehicle maintained in a direction sensitive vector of known to be in track vehicles.  The preservation of observed information enables predictive filter
// based tracking, done elsewhere.
//
//  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  * 

bool manageMovers(Mat wholeScenethreshImage, Mat &AnalysisFrame){

/// < < < < < < < < < < < < < < < < < < < < < < < < < < G e t   P r o j e c t i o n s   f o r   v e h s   a l r e a d y   i n   t r a c k  > > > > > > > > > > > > > > > > 
// Get all L2R vehicle projections
	vector<Projection> projectedL2R;  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	for (int index = 0; index < vehiclesGoingRight.size(); index++){
		projectedL2R.push_back(vehiclesGoingRight[index].getBestProjection(g, frameNumber));
		if (pleaseTrace) traceFile << endl << "<" << frameNumber << "> Project >>L2R>> vehicle[" << index << "]  Rect xywh: [" << projectedL2R[index].getBox().x << ", "
			<< projectedL2R[index].getBox().y << ", " << projectedL2R[index].getBox().width << ",  " << projectedL2R[index].getBox().height
			<< "]   vState: " << vStateString(projectedL2R[index].getVState())
			<< "  Overlap: " << overlapString(vehiclesGoingRight[index].getOverlapStatus())
			<< ",   pixDelta: " << projectedL2R[index].getVelocity() << endl
			<< "     FBSlope (+): " << vehiclesGoingRight[index].getFBSlope() << "   FBIntcpt:  " << vehiclesGoingRight[index].getFBIntercept()
			<< "  RBSlope: " << vehiclesGoingRight[index].getRBSlope() << "  RBIntcpt: " << vehiclesGoingRight[index].getRBIntercept()
			<< "      projected FB: " << int(vehiclesGoingRight[index].getNextFrontBumper())
			<< "  projected width: " << projectedL2R[index].getBox().width 
			<< "  projected RB: " << int(vehiclesGoingRight[index].getNextRearBumper())
			<< endl;
	}

// Get all R2L vehicle projections
	vector<Projection> projectedR2L;  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	for (int index = 0; index < vehiclesGoingLeft.size(); index++){
		projectedR2L.push_back(vehiclesGoingLeft[index].getBestProjection(g, frameNumber));
		if (pleaseTrace) traceFile << endl << "<" << frameNumber << "> Project <<R2L<< vehicle[" << index << "]  Rect xywh: [" << projectedR2L[index].getBox().x << ", "
			<< projectedR2L[index].getBox().y << ", " << projectedR2L[index].getBox().width << ",  " << projectedR2L[index].getBox().height
			<< "]   vState: " << vStateString(projectedR2L[index].getVState())
			<< "  Overlap: " << overlapString(vehiclesGoingLeft[index].getOverlapStatus())
			<< ",   pixDelta: " << projectedR2L[index].getVelocity() << endl
			<< "     FBSlope (-): " << vehiclesGoingLeft[index].getFBSlope() << "   FBIntcpt:  " << vehiclesGoingLeft[index].getFBIntercept()
			<< "  RBSlope: " << vehiclesGoingLeft[index].getRBSlope() << "  RBIntcpt: " << vehiclesGoingLeft[index].getRBIntercept()
			<< "      projected FB: " << int(vehiclesGoingLeft[index].getNextFrontBumper())
			<< "  projected width: " << projectedR2L[index].getBox().width 
			<< "      projected RB: " << int(vehiclesGoingLeft[index].getNextRearBumper())
			<< endl;
	}


//  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  Get rid of all exited and deleted vehicles *  *  *  *  *  *  *  *  *  * 

// If front L2R vehicle is exited, remove it from consideration
	if ((vehiclesGoingRight.size() > 0) && (projectedL2R.front().getVState() == exited)){
		if (pleaseTrace) traceFile << endl << "<" << frameNumber << ">   # # # # # # # L2R vehicle just exited." << endl;
//		cout << "<" << frameNumber << ">   # # # # # # # L2R vehicle just exited." << endl;
		logL2Rstats(true, 0); //
		vehiclesGoingRight.erase(vehiclesGoingRight.begin());
		projectedL2R.erase(projectedL2R.begin());
	}

// If front R2L vehicle is exited, remove it from consideration
	if ((vehiclesGoingLeft.size() > 0) && (projectedR2L.front().getVState() == exited)){
		if (pleaseTrace) traceFile << endl << "<" << frameNumber << ">   # # # # # # # R2L vehicle just exited." << endl;
//		cout << "<" << frameNumber << ">   # # # # # # # R2L vehicle just exited." << endl;
		logR2Lstats(true, 0);
		vehiclesGoingLeft.erase(vehiclesGoingLeft.begin());
		projectedR2L.erase(projectedR2L.begin());
	}


// Check for deleted L2R vehicles

	for (int index = vehiclesGoingRight.size() - 1; index > -1; index--){
		if (vehiclesGoingRight[index].getAmIOK() != ImOK) {
			if (pleaseTrace) traceFile << endl << "<" << frameNumber << ">   # # # # # # # L2R vehicle[" << index << "] is being deleted: " << statusString(vehiclesGoingRight[index].getAmIOK()) << endl;
//			cout << "<" << frameNumber << ">   # # # # # # # L2R vehicle[" << index << "] is being deleted: " << statusString(vehiclesGoingRight[index].getAmIOK()) << endl;
			if (vehiclesGoingRight[index].getTrackEndPixel() > 0) logL2Rstats(true, index);
			else logL2Rstats(false, /*vehiclesGoingRight[index].getAmIOK() == deleteWithStats*/ index);
			vehiclesGoingRight.erase(vehiclesGoingRight.begin() + index);
			projectedL2R.erase(projectedL2R.begin() + index);
		}
	}

// Check for deleted R2L vehicles

	for (int index = vehiclesGoingLeft.size() - 1; index > -1; index--){
		if (vehiclesGoingLeft[index].getAmIOK() != ImOK) {
			if (pleaseTrace) traceFile << endl << "<" << frameNumber << ">   # # # # # # # R2L vehicle[" << index << "] is being deleted: " << statusString(vehiclesGoingLeft[index].getAmIOK()) << endl;
//			cout << "<" << frameNumber << ">   # # # # # # # R2L vehicle[" << index << "] is being deleted: " << statusString(vehiclesGoingLeft[index].getAmIOK()) << endl;
			if (vehiclesGoingLeft[index].getTrackEndPixel() > 0) logR2Lstats(true, index);
			else logR2Lstats(false, /*vehiclesGoingLeft[index].getAmIOK() == deleteWithStats,*/ index);
			vehiclesGoingLeft.erase(vehiclesGoingLeft.begin() + index);
			projectedR2L.erase(projectedR2L.begin() + index);
		}
	}

// Check for L2R overrunning, as in a vehicle starting to pass a bicyclist; bail if overrunning detected.
// This could be modified to delete the overrun vehicle instead, but leapfrogging would have to be dealt with.

	for (int index = vehiclesGoingRight.size() - 1; index > 0; index--){
		if (index > 0 && (projectedL2R[index].getBox().x + projectedL2R[index].getBox().width) > (projectedL2R[index - 1].getBox().x - 200) ) {
			if (pleaseTrace) traceFile << endl << "<" << frameNumber << ">   # # # # # # # L2R vehicle[" << index << "] is being deleted for overrunning: " << endl;
			cout << "<" << frameNumber << ">   # # # # # # # L2R vehicle[" << index << "] is overrunning: "  << endl;
			// For now, erase all ongoing vehicle records and wait for scene to go quiescent.  Then start analyzing again.
			vehiclesGoingRight.erase(vehiclesGoingRight.begin(), vehiclesGoingRight.end());
			vehiclesGoingLeft.erase(vehiclesGoingLeft.begin(), vehiclesGoingLeft.end());
			projectedL2R.erase(projectedL2R.begin(), projectedL2R.end());
			projectedR2L.erase(projectedR2L.begin(), projectedR2L.end());
			bailing = true;
			cv::line(AnalysisFrame, Point(g.pixelLeft + 1, 20), Point(g.pixelRight - 1, 20), Scalar(CVRed), 2);
			if (pleaseTrace) traceFile << "<" << frameNumber << "> Starting to bail because of overrunning.   All current vehicles being dropped." << endl;
		}
	}

// Check for R2L overrunning, as in a vehicle starting to pass a bicyclist; bail if overrunning detected.
// This could be modified to delete the overrun vehicle instead, but leapfrogging would have to be dealt with.

	for (int index = vehiclesGoingLeft.size() - 1; index > 0; index--){
		if (index > 0 && ((projectedR2L[index - 1].getBox().x + projectedR2L[index - 1].getBox().width) > (projectedR2L[index].getBox().x - 200))) {
			if (pleaseTrace) traceFile << endl << "<" << frameNumber << ">   # # # # # # # R2L vehicle[" << index << "] is being deleted for overrunning: " << endl;
			cout << "<" << frameNumber << ">   # # # # # # # R2L vehicle[" << index << "] is overrunning: " << endl;
			// For now, erase all ongoing vehicle records and wait for scene to go quiescent.  Then start analyzing again.
			vehiclesGoingRight.erase(vehiclesGoingRight.begin(), vehiclesGoingRight.end());
			vehiclesGoingLeft.erase(vehiclesGoingLeft.begin(), vehiclesGoingLeft.end());
			projectedL2R.erase(projectedL2R.begin(), projectedL2R.end());
			projectedR2L.erase(projectedR2L.begin(), projectedR2L.end());
			bailing = true;
			cv::line(AnalysisFrame, Point(g.pixelLeft + 1, 20), Point(g.pixelRight - 1, 20), Scalar(CVRed), 2);
			if (pleaseTrace) traceFile << "<" << frameNumber << "> Starting to bail because of overrunning.   All current vehicles being dropped." << endl;
		}
	}
	


//  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  Detect places of motion  *  *  *  *  *  *  *  *  *  * 
	vector< vector<Point> > contours; // for findContours output
	vector<Vec4i> hierarchy;  // for findContours output

	int numOKSizeObjectsL2R = 0; // Used to count how many detected objects are in selected region of interest
	Mat tempForL2R = wholeScenethreshImage(Rect(g.pixelLeft, 0, g.pixelRight, g.L2RStreetY)); // L2RStreetY is the lowest needed to go to see a rightbound vehicle
	Rect objectBoundingRectangleL2R[MAX_NUM_OBJECTS]; // bounding rectangles, from top of ROI to L2R lane, captured in a given frame

	findContours(tempForL2R, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
	// found some objects?
	if (contours.size() > 0){   // Are both of
		if (hierarchy.size() > 0) {  // these necessary?
			numObjects = hierarchy.size();
			//if number of objects greater than MAX_NUM_OBJECTS may need to adjust noise filter
			if (numObjects < MAX_NUM_OBJECTS){
				for (int index = 0; index >= 0; index = hierarchy[index][0]) {
					objectBoundingRectangleL2R[numOKSizeObjectsL2R] = boundingRect(contours.at(index)); 		//make bounding rectangle 
					if ((objectBoundingRectangleL2R[numOKSizeObjectsL2R].width * objectBoundingRectangleL2R[numOKSizeObjectsL2R].height) >= MIN_OBJECT_AREA)
						numOKSizeObjectsL2R++;
				} // for
			} // if
			else {
				cout << "Too many L2R objects!" << endl;
				numOKSizeObjectsL2R = 0;
			}
		} // if
	} // if


	int numOKSizeObjectsR2L = 0; // Used to count how many detected objects are in selected region of interest
	Mat tempForR2L = wholeScenethreshImage(Rect(g.pixelLeft, 0, g.pixelRight, g.R2LStreetY)); // R2LStreetY is the lowest needed to go for leftbound vehicle
	Rect objectBoundingRectangleR2L[MAX_NUM_OBJECTS]; // bounding rectangles captured in a given frame

	contours.erase(contours.begin(), contours.end()); // Clear contours vector
	hierarchy.erase(hierarchy.begin(), hierarchy.end());  // Clear hierarchy vector
	//find external contours of filtered image using openCV findContours function
	findContours(tempForR2L, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
	// found some objects?
	if (contours.size() > 0){   // Are both of
		if (hierarchy.size() > 0) {  // these necessary?
			numObjects = hierarchy.size();
			//if number of objects greater than MAX_NUM_OBJECTS may need to adjust noise filter
			if (numObjects < MAX_NUM_OBJECTS){
				for (int index = 0; index >= 0; index = hierarchy[index][0]) {
					objectBoundingRectangleR2L[numOKSizeObjectsR2L] = boundingRect(contours.at(index)); 		//make bounding rectangle 
					if ((objectBoundingRectangleR2L[numOKSizeObjectsR2L].width * objectBoundingRectangleR2L[numOKSizeObjectsR2L].height) >= MIN_OBJECT_AREA)
						numOKSizeObjectsR2L++;
				} // for
			} // if
			else {
				cout << "Too many R2L objects!" << endl;
				numOKSizeObjectsR2L = 0;
			}
		} // if
	} // if



// At this point objectBoundingRectanglexxx[] has numOKSizeObjectsxxx acceptable rectangles in it, possibly zero.  The rectangles are independent, non-overlapping.


// This bailing code is used in circumstances where the scene is overwhelming.

	if (((numOKSizeObjectsL2R + numOKSizeObjectsR2L) > 0) && bailing){
		cv::line(AnalysisFrame, Point(g.pixelLeft + 1, 20), Point(g.pixelRight - 1, 20), Scalar(CVRed), 2);
		if (pleaseTrace) traceFile << "<" << frameNumber << "> Still bailing." << endl;
		return true;
	}
	else if(bailing){ // bailing with no objects detected.
		bailing = false;
		if (pleaseTrace) traceFile << "<" << frameNumber << "> Returning to analyzing traffic." << endl;
	}



//  Visual bracketing of the left/right ends of the range where speed measuring takes place.

	cv::line(AnalysisFrame, Point(g.speedLineLeft, 180), Point(g.speedLineLeft, 25), Scalar(CVWhite), 2);
	cv::line(AnalysisFrame, Point(g.speedLineRight, 180), Point(g.speedLineRight, 25), Scalar(CVWhite), 2);


	if ((numOKSizeObjectsL2R + numOKSizeObjectsR2L) > 0) { // rectangles found in areas checked, i.e. motion detected;  See what's up...

//		cout << "<" << frameNumber << "> Num OK objects: " << numOKSizeObjects << "  L2R vehicles: " << vehiclesGoingRight.size() << "  R2L vehicles: " << vehiclesGoingLeft.size() << endl;
		if (pleaseTrace) traceFile << "<" << frameNumber << "> Num OK L2R objects: " << numOKSizeObjectsL2R
			<< " Num OK R2L objects: " << numOKSizeObjectsR2L << "  L2R vehicles: " << vehiclesGoingRight.size() << "  R2L vehicles: " << vehiclesGoingLeft.size() << endl;


//  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  Safe to look for newly entering vehicles at the left and right ends of the analysis box? *  *  *  *  *  *  *  *
//                                                       =========================================================================================

// Left end is safe to check if there are no entering L2R vehicles and no R2L vehicles w/in a couple of frames of exiting left.
//  > > > > > > > > > > > > > > > > > 
		int safeL2RZone = g.pixelRight - g.pixelLeft;
		int safeR2LZone = g.pixelRight - g.pixelLeft;
		// "6" and "2" in following if statements can be tweaked.  I'm happy with their current values.
		if (projectedR2L.size() > 0) safeR2LZone = max(projectedR2L.front().getBox().x - (2 * g.maxL2RDistOnEntry), 0);  // Identify safe range to front of oncoming car.
		if (projectedL2R.size() > 0) safeL2RZone = max(projectedL2R.back().getBox().x - (6 * g.maxL2RDistOnEntry), 0);  // Identify safe range to rear of preceding car.

		if (safeL2RZone > 0 && safeR2LZone > 0){
			coalescedRectangle = coalesce(objectBoundingRectangleL2R, numOKSizeObjectsL2R,
				g.pixelLeft, min(min(safeR2LZone, safeL2RZone), (g.pixelLeft + g.pixelRight) / 2), strict);  // Look for vehicle from left (-20 covers projection slop)
			if (coalescedRectangle.x != -1){ // at least one object is present in coalesced rectangle(s)
				vehiclesGoingRight.push_back(VehicleDynamics(L2R));
				vehiclesGoingRight[vehiclesGoingRight.size() - 1].addSnapshot(Snapshot(coalescedRectangle, frameNumber));
				if (coalescedRectangle.x + coalescedRectangle.width >= g.speedLineLeft)
					     vehiclesGoingRight[vehiclesGoingRight.size() - 1].markInvalidSpeed();
				if (pleaseTrace) traceFile << endl << endl << "<" << frameNumber
					<< ">    >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Just added rightbound vehicle[" << vehiclesGoingRight.size() - 1 << "] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
				displayAnalysisGoingRight(frameNumber, vehiclesGoingRight.size() - 1, coalescedRectangle, none, AnalysisFrame, -1);
			}
		}

// Right end is safe if there are no entering R2L vehicles and no L2R vehicles w/in a couple of frames of exiting left.
// < < < < < < < < < < < < < < < < < < 
		safeL2RZone = g.pixelRight - g.pixelLeft;
		safeR2LZone = g.pixelRight - g.pixelLeft;
		 // "6" and "2" in following if statements can be tweaked.  I'm happy with their current values.
		if (projectedR2L.size() > 0)  safeR2LZone = max(g.pixelRight - (projectedR2L.back().getBox().x + projectedR2L.back().getBox().width + (6 * g.maxR2LDistOnEntry)), 0);   // Identify safe range to rear of preceding car.
		if (projectedL2R.size() > 0) safeL2RZone = max(g.pixelRight - (projectedL2R.front().getBox().x + projectedL2R.front().getBox().width + (2 * g.maxR2LDistOnEntry)), 0);  // Identify safe range to front of oncoming car.

		if (safeR2LZone > 0 && safeL2RZone > 0){
			coalescedRectangle = coalesce(objectBoundingRectangleR2L, numOKSizeObjectsR2L,
				     max(  max(g.pixelRight - safeL2RZone, g.pixelRight - safeR2LZone),
				          (g.pixelLeft + g.pixelRight) / 2), g.pixelRight, strict);  // Look for vehicle from right
			if (coalescedRectangle.x != -1){
				vehiclesGoingLeft.push_back(VehicleDynamics(R2L));
				vehiclesGoingLeft[vehiclesGoingLeft.size() - 1].addSnapshot(Snapshot(coalescedRectangle, frameNumber));
				if (coalescedRectangle.x <= g.speedLineRight)
					     vehiclesGoingLeft[vehiclesGoingLeft.size() - 1].markInvalidSpeed();
				if (pleaseTrace) traceFile << endl << endl << "<" << frameNumber
					<< ">     <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Just added leftbound vehicle[" << vehiclesGoingLeft.size() - 1 << "] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
				displayAnalysisGoingLeft(frameNumber, vehiclesGoingLeft.size() - 1, coalescedRectangle, none, AnalysisFrame, -1);
			}
		}


// * * * * * * * * * * * * * * * * * * * * * * * *  P r o c e s s    a l l    p r o j e c t e d    v e h i c l e s * * * * * * * * * * * * * * * * * * * *
//                                                 ================================================================

// ---------------More than three in analysis zone with opposing traffic......
		if ((vehiclesGoingRight.size() * vehiclesGoingLeft.size()) > 2){ // Bail on mixed direction, four or more vehicles total (includes just entered vehs)
			// For now, erase all ongoing vehicle records and wait for scene to go quiescent.  Then start analyzing again.
			vehiclesGoingRight.erase(vehiclesGoingRight.begin(), vehiclesGoingRight.end());
			vehiclesGoingLeft.erase(vehiclesGoingLeft.begin(), vehiclesGoingLeft.end());
			projectedL2R.erase(projectedL2R.begin(), projectedL2R.end());
			projectedR2L.erase(projectedR2L.begin(), projectedR2L.end());
			bailing = true;
			cv::line(AnalysisFrame, Point(g.pixelLeft + 1, 20), Point(g.pixelRight - 1, 20), Scalar(CVRed), 2);
			if (pleaseTrace) traceFile << "<" << frameNumber << "> Starting to bail because  > 3 bi-directional traffic detected.  All current vehicles being dropped." << endl;
		}


		else {  // Ok, scene is one that can be handled. Clear past info about passing vehicles, and then check for passing vehicles now.
			if(vehiclesGoingRight.size() > 0)
				for (int i = 0; i < vehiclesGoingRight.size(); i++) 
					vehiclesGoingRight[i].setOverlapStatus(none); // Reset any past L2R overlap determinations.
			if (vehiclesGoingLeft.size() > 0)
				for (int i = 0; i < vehiclesGoingLeft.size(); i++)
					vehiclesGoingLeft[i].setOverlapStatus(none); // Reset any past R2L overlap determinations.
// -------------- Two or three vehicles in analysis zone, a least one in each direction......
			if ((projectedL2R.size() * projectedR2L.size()) >= 1) { // Two vehs currently in track (ignoring just entered vehicles now), one in each direction;
				for (int i = 0; i < projectedL2R.size(); i++)
					vehiclesGoingRight[i].setOverlapStatus(doesL2ROverlapAnyR2L(i, projectedL2R, projectedR2L, projectedR2L.size()));
				for (int i = 0; i < projectedR2L.size(); i++)
					vehiclesGoingLeft[i].setOverlapStatus(doesR2LOverlapAnyL2R(i, projectedR2L, projectedL2R, projectedL2R.size()));
			}

		}

// > > > > > > > > > > > >  All *current* vehicles from left case (any newly added L2R vehicle not considered) > > > > > > > > > > > > > > > > > > > 
//		                   ===================================================================================
//  > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > > >

		Rect objectBoundingRectangle[MAX_NUM_OBJECTS]; // bounding rectangles, formed in a direction sensitive manner

		if (projectedL2R.size() > 0){ // All bidirectional cases considered by the time control gets here.
			for (int index = 0; index < projectedL2R.size(); index++){
            // First, focus the search for detected blobs to the region the vehicle is projected to occupy
				//find external contours of filtered image using openCV findContours function
				contours.erase(contours.begin(), contours.end()); // Clear contours vector
				hierarchy.erase(hierarchy.begin(), hierarchy.end());  // Clear hierarchy vector
				int tempX = max(projectedL2R[index].getBox().x - 80, g.pixelLeft);  // look behind the predicted rear bumper
				int tempWidth = min(projectedL2R[index].getBox().width + 100, g.pixelRight - tempX); // Look a little beyond the front bumper;
				Mat ROIL2R = wholeScenethreshImage(Rect(tempX, 0, tempWidth, g.L2RStreetY));
				findContours(ROIL2R, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
				int numOKSizeL2RObjects = 0; // Used to count how many detected objects are in selected region of interest
				// found some objects?
				if (contours.size() > 0){   // Are both of
					if (hierarchy.size() > 0) {  // these necessary?
						for (int index = 0; index >= 0; index = hierarchy[index][0]) {
							objectBoundingRectangle[numOKSizeL2RObjects] = boundingRect(contours.at(index)); 		//make bounding rectangle 
							if ((objectBoundingRectangle[numOKSizeL2RObjects].width * objectBoundingRectangle[numOKSizeL2RObjects].height) >= MIN_OBJECT_AREA){
								objectBoundingRectangle[numOKSizeL2RObjects].x += tempX;
								numOKSizeL2RObjects++;
							}
						} // for
					} // if
				} // if
			if(pleaseTrace) traceFile << "    Number of L2R objects is: " << numOKSizeL2RObjects << "  inside rect[x,y,wid,ht] "
				<< tempX << ", " << 0 << ", " << tempWidth << ", " << g.L2RStreetY << endl;

			// Get the best bounding rectangle possible for the vehicle being considered; if no objects were found, skip to display of projected data
				if (numOKSizeL2RObjects > 0){
					int projFrontBumper = projectedL2R[index].getBox().x + projectedL2R[index].getBox().width;
					int projRearBumper = projectedL2R[index].getBox().x;
					grabType grabRestriction = greedy;
					if (vehiclesGoingRight[index].getOverlapStatus() == rearOnly)
						grabRestriction = strict;
					if (projectedL2R[index].getVState() == entering)
						// Look a few pixels beyond projections in each direction
						coalescedRectangle = coalesce(objectBoundingRectangle, numOKSizeL2RObjects, max(g.pixelLeft, (projRearBumper - 10)), (projFrontBumper + 20), grabRestriction);
					else if (projectedL2R[index].getVState() == exiting)  // Look a few pixels beyond projections in each direction
						coalescedRectangle = coalesce(objectBoundingRectangle, numOKSizeL2RObjects, (projRearBumper - 10), min((projFrontBumper + 10), g.pixelRight), grabRestriction);
					else  // somewhere in the middle
						coalescedRectangle = coalesce(objectBoundingRectangle, numOKSizeL2RObjects, max((projRearBumper - 50),
						g.pixelLeft), min((projFrontBumper + 10), g.pixelRight), grabRestriction);

					// If no coalesced objects have been found, record no snapshot.
					//                                        =======================
					if (coalescedRectangle.x >= 0){    					// Coalesced objects found...
						vehiclesGoingRight[index].addSnapshot(Snapshot(coalescedRectangle, frameNumber));
						// draw a purple rectangle around the area where objects related to the vehicle were found ("actual data")
						line(AnalysisFrame, Point(coalescedRectangle.x, coalescedRectangle.y), Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y), Scalar(CVPurple), 2);
						line(AnalysisFrame, Point(coalescedRectangle.x, coalescedRectangle.y + coalescedRectangle.height),
							Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y + coalescedRectangle.height), Scalar(CVPurple), 2);
						line(AnalysisFrame, Point(coalescedRectangle.x, coalescedRectangle.y), Point(coalescedRectangle.x, coalescedRectangle.y + coalescedRectangle.height), Scalar(CVPurple), 2);
						line(AnalysisFrame, Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y),
							Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y + coalescedRectangle.height), Scalar(CVPurple), 2);
						if (pleaseTrace) traceFile << "  Observed FB: " << coalescedRectangle.x + coalescedRectangle.width
							<< "   Observed width: " << coalescedRectangle.width 
							<< "  Observed RB: " << coalescedRectangle.x
							<< endl;
					}
				}

				displayAnalysisGoingRight(frameNumber, index, projectedL2R[index].getBox(), vehiclesGoingRight[index].getOverlapStatus(), AnalysisFrame, vehiclesGoingRight[index].getFinalSpeed());
			}
		}

//  < < < < < < < < < < < < < < < <   All *current* vehicles from right case (any newly added R2L vehicle not considered) < < < < < < < < < < < < < < < < < < < < < <
//		                              ===================================================================================
//  < < < < < < < < < < < < < < < <  < < < < < < < < < < < < < < < <  < < < < < < < < < < < < < < < <  < < < < < < < < < < < < < < < <  < < < < < < < < < < < < < < < < 


		if (0 < projectedR2L.size()) { 
			for (int index = 0; index < projectedR2L.size(); index++){
				// First, focus the search for detected blobs to the region the vehicle is projectyed to occupy
				//find external contours of filtered image using openCV findContours function
				contours.erase(contours.begin(), contours.end()); // Clear contours vector
				hierarchy.erase(hierarchy.begin(), hierarchy.end());  // Clear hierarchy vector
				int tempX = max(projectedR2L[index].getBox().x - 20, g.pixelLeft);  // look a little ahead of the predicted front bumper
				int tempWidth = min(projectedR2L[index].getBox().width + 100, g.pixelRight - tempX); // Look behind the rear bumper;
				Mat ROIR2L = wholeScenethreshImage(Rect(tempX, 0, tempWidth, g.R2LStreetY));
				findContours(ROIR2L, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
				int numOKSizeR2LObjects = 0; // Used to count how many detected objects are in selected region of interest
				// found some objects?
				if (contours.size() > 0){   // Are both of
					if (hierarchy.size() > 0) {  // these necessary?
						for (int index = 0; index >= 0; index = hierarchy[index][0]) {
							objectBoundingRectangle[numOKSizeR2LObjects] = boundingRect(contours.at(index)); 		//make bounding rectangle 
							if ((objectBoundingRectangle[numOKSizeR2LObjects].width * objectBoundingRectangle[numOKSizeR2LObjects].height) >= MIN_OBJECT_AREA){
								objectBoundingRectangle[numOKSizeR2LObjects].x += tempX;
								numOKSizeR2LObjects++;
							}
						} // for
					} // if
				} // if
				if (pleaseTrace) traceFile << "    Number of R2L objects is: " << numOKSizeR2LObjects << "  inside rect[x,y,wid,ht] "
					<< tempX << ", " << 0 << ", " << tempWidth << ", " << g.R2LStreetY << endl;



				// Get the best bounding rectangle possible
				if (numOKSizeR2LObjects > 0){ // Get the best bounding rectangle possible for the vehicle being considered; if no objects were found, skip to display of projected data
					int projFrontBumper = projectedR2L[index].getBox().x;
					int projRearBumper = projFrontBumper + projectedR2L[index].getBox().width;
					grabType grabRestriction = greedy;
					if (vehiclesGoingLeft[index].getOverlapStatus() == rearOnly)
						grabRestriction = strict;
					if (projectedR2L[index].getVState() == entering)
						// Look a few pixels beyond projections in each direction
						coalescedRectangle = coalesce(objectBoundingRectangle, numOKSizeR2LObjects, (projFrontBumper - 20), min((projRearBumper + 10), g.pixelRight), grabRestriction); // minus for R2L vehicle
					else if (projectedR2L[index].getVState() == exiting)  // Look a few pixels beyond projections in each direction
						coalescedRectangle = coalesce(objectBoundingRectangle, numOKSizeR2LObjects, max((projFrontBumper - 10), g.pixelLeft), (projRearBumper + 10), grabRestriction);
					else  // somewhere in the middle
						coalescedRectangle = coalesce(objectBoundingRectangle, numOKSizeR2LObjects, max((projFrontBumper - 20), g.pixelLeft),
						min(projRearBumper + 50, g.pixelRight), grabRestriction);

					// If no coalesced objects have been found, record no snapshot.
					//                                        =======================
					if (coalescedRectangle.x >= 0){  					// Coalesced objects found...
						vehiclesGoingLeft[index].addSnapshot(Snapshot(coalescedRectangle, frameNumber));
						// draw an orange rectangle around the area where objects related to the vehicle were found ("actual data")
						line(AnalysisFrame, Point(coalescedRectangle.x, coalescedRectangle.y), Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y), Scalar(CVOrange), 2);
						line(AnalysisFrame, Point(coalescedRectangle.x, coalescedRectangle.y + coalescedRectangle.height),
							Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y + coalescedRectangle.height), Scalar(CVOrange), 2);
						line(AnalysisFrame, Point(coalescedRectangle.x, coalescedRectangle.y), Point(coalescedRectangle.x, coalescedRectangle.y + coalescedRectangle.height), Scalar(CVOrange), 2);
						line(AnalysisFrame, Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y),
							Point(coalescedRectangle.x + coalescedRectangle.width, coalescedRectangle.y + coalescedRectangle.height), Scalar(CVOrange), 2);
						if (pleaseTrace) traceFile << "  Observed FB: " << coalescedRectangle.x
							<< "   Observed width: " << coalescedRectangle.width 
							<< "   Observed RB: " << coalescedRectangle.x + coalescedRectangle.width
							<< endl;
					}
				}
				displayAnalysisGoingLeft(frameNumber, index, projectedR2L[index].getBox(), vehiclesGoingLeft[index].getOverlapStatus(), AnalysisFrame, vehiclesGoingLeft[index].getFinalSpeed());

			}
		}
	}

	else; // cout << "<" << frameNumber << ">  . . ." << endl; // This happens if numOKObjects == 0;

	return ((numOKSizeObjectsL2R + numOKSizeObjectsR2L) > 0);
}


// ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^  M a i n  ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ 
// ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^  M a i n  ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ 
//
//  Process image data per user request.  Key call halfway down is this:
//                                                    objectDetected = manageMovers(thresholdImage, ROIFr2);
// which causes processing of all known and newly entered vehicls to occur at time "frameNumber."  This one call exercises most of the code above
// and most all of the code in vehicleDynamics.

int main(){

	bool objectDetected = false;
	bool pause = false;  	 // toggle using "p"
	bool showVideo = true;  // turning this off should make processing run faster.  toggled with a "v"
	Mat grayImage1, grayImage2; // for absdiff() function
	Mat differenceImage;
	Mat thresholdImage;  	//thresholded difference image (for use in findContours() function)

	setup();  // Get config data and user preferences for files to process, tracing, debugging, start frame and others

//  * * * * * * * * * * * * * * * * * * * * * *  M a i n   L o o p   o v e r   o n e   o r   m o r e   i n p u t   f i l e s  * * * * * * * * * * * * * * * *


	while (moreFilesToDo){

		if (yesNoAll == "*"){
			if (getline(filesList, fileName)){
				cout << endl << "Now processing cam input file: " << fileName << endl;
				if (pleaseTrace) traceFile << endl << "Now processing cam input file: " << fileName << endl;
				vehiclesGoingRight.erase(vehiclesGoingRight.begin(), vehiclesGoingRight.end());  // Reinitialize
				vehiclesGoingLeft.erase(vehiclesGoingLeft.begin(), vehiclesGoingLeft.end());   // Reinitialize  
				bailing = false;  // Reinitialize
				startFrame = 0.0;
			}
			else{
				cout << endl << "Done processing all input files: " << fileName << endl;
				if (pleaseTrace) traceFile << endl << "Done processing all input files: " << fileName << endl;
				moreFilesToDo = false;
				filesList.close();
				break; 
			}
		}
		else {  // yesNoAll must == "y";  "fileName" is the name of the file to process
			moreFilesToDo = false;
			filesList.close();
		}


		// dirName is the directory name (only) in which input files reside.  Its name is expected to be of the form: yyyymmdd
		// dirPath is the path to the directory in which input (.avi) files are located; it includes dirName at the end, but no trailing reverse slashes 
		// fileName is the name of the current avi file to be processed.  Its form is "manual_" <yyyymmddhhmmss> ".avi"   <<-- no spaces

		string FName = dirPath + "\\" + fileName;
		cout << "Trying to capture from " + FName << endl;
		capture.open(FName);

		if (!capture.isOpened()){
			cout << "ERROR ACQUIRING VIDEO FEED\n";
			getchar();
			return -1;
		}

		double frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
		if (frameWidth != 1280.0){
			cout << "Frame width is not 1280.  Bailing";
			return - 1;
		}
		double FPS = capture.get(CV_CAP_PROP_FPS);
		if (FPS != 30.0){
			cout << "Frame rate is not 30.  Bailing";
			return -1;
		}

		capture.set(CV_CAP_PROP_POS_FRAMES, startFrame);  // Set frame number to start at, in first file to be processed;  Remaining files will start at zero.
		frameNumber = int(startFrame);
		int delay = 10;   //at least 10ms delay is necessary for proper operation of this program <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		//work through frame pairs looking for differences
		while (capture.get(CV_CAP_PROP_POS_FRAMES) < capture.get(CV_CAP_PROP_FRAME_COUNT) - 2){ // minus 2 to prevent reading empty frame at end.
			capture.read(frame1);
			Mat ROIFr1 = frame1(AnalysisBox).clone();  			// Carve out the analysis box for motion detection
			cv::cvtColor(ROIFr1, grayImage1, COLOR_BGR2GRAY);  //convert ROIFr1 to gray scale for frame differencing
			capture.read(frame2);
			Mat ROIFr2 = frame2(AnalysisBox).clone();   		   // Carve out the analysis box for motion detection
			cv::cvtColor(ROIFr2, grayImage2, COLOR_BGR2GRAY);   //convert ROIFr2 to gray scale for frame differencing
			cv::absdiff(grayImage1, grayImage2, differenceImage);   			//perform frame differencing
			cv::threshold(differenceImage, thresholdImage, g.SENSITIVITY_VALUE, 255, THRESH_BINARY);  //threshold intensity image at a given sensitivity value
			cv::blur(thresholdImage, thresholdImage, cv::Size(g.BLUR_SIZE, g.BLUR_SIZE));  //blur the image to reduce noise.
			cv::threshold(thresholdImage, thresholdImage, g.SENSITIVITY_VALUE, 255, THRESH_BINARY);	//threshold again to obtain binary image from blur output

			if (showVideo)	imshow("Final Threshold Image", thresholdImage);
			else cv::destroyWindow("Final Threshold Image");

		// ************************************************* Vehicle motion analysis *****************************************************
			objectDetected = manageMovers(thresholdImage, ROIFr2);

			frameNumber += 2;  // Note: frames are used in frame differencing operations only once each, so frame count jumps by two, not one.
			                  // One could argue that using each frame as the second frame in a differencing operation, and then using it a second time
			                  // as the first frame in the next differencing operation would increase resolution.  It probably would.  However,
			                  // doubling frame differencing operations will increase processing times, and probably not add much to speed estimations quality.
			                  // Look at the function computeFinalSpeed() in vehicleDynamics.cpp where I analyze distances of front bumper from the speed zone
			                  // lines to decide whether or not to add or subtract one frame from the total number of frames a vehicle took to pass
			                  // through the speed measuring zone.   I contend performing the +/-1 analysis brings back the accuracy that doubling frame
			                  // differencing operations would provide, but at half the computational cost.

			//show captured frame
			 if (showVideo)imshow("Whole Scene", ROIFr2);

			if (!showVideo)
				delay = 1;
			else if (objectDetected) 
				delay = objDelay;
			else 
				delay = 10;

			switch (waitKey(delay)){
			case 27: //'esc'     exit program.
				return 0;
			case 102: // 'f'    make display go faster;
				if (objDelay > 10) objDelay = objDelay / 5;
				cout << "<" << frameNumber << ">  Delay:" << objDelay << endl;
				break;
			case 112: //'p'     pause/resume.
				pause = !pause;
				if (pause == true){
					cout << "Code paused, press 'p' again to resume" << endl;
					while (pause == true){
						//wait for another p
						switch (waitKey()){
						case 112:
							pause = false;
							cout << "<" << frameNumber << ">  Code Resumed" << endl;
							break;
						} // switch
					} // while paused
				} // if pause
				break;
			case 115: // 's'      slow down display rate
				if (objDelay <1250) objDelay = 5* objDelay;
				cout << "<" << frameNumber << ">  Delay:" << objDelay << endl;
				break;
			case 118:  // 'v'  turn video on/off
				showVideo = !showVideo;
				break;
			} // switch

		} // main loop for processing one input file

		capture.release();

	} // looping over input files loop end

//	if (highLightsPlease) hiLiteVideo.release();
	if (pleaseTrace) traceFile.close();
	if (highLightsPlease) hiLiteVideo.release();
	statsFile.close();
	return 0;

}

