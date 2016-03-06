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

#pragma once
#include "Globals.h"
//#include <string>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include "Projection.h"
#include "Snapshot.h"

using namespace std;
using namespace cv;

class VehicleDynamics
{
public:

	VehicleDynamics();

	VehicleDynamics(direction dir);

	~VehicleDynamics();



	void addSnapshot(Snapshot inShot);
	int computeFinalSpeed(Globals&, direction, int, int, int, int, int, int, double);
	Projection getBestProjection(Globals& g, int framenum);

	int getTrackStartPixel();
	int getTrackEndPixel();
	int getTrackStartFrame();
	int getTrackEndFrame();

	int getArea();

	double getFBSlope();

	double getFBIntercept();

	double getRBSlope();

	double getRBIntercept();

	double getNextFrontBumper();

	double getNextRearBumper();

	statusTypes getAmIOK();

	void setOverlapStatus(OverlapType inOverlapStatus);

	OverlapType getOverlapStatus();

	void markInvalidSpeed();

	int getFinalSpeed();

	void holdFrame(Mat inMat);

	void saveFrame(Mat inMat);

	Mat getHeldFrame();

	Mat getSavedFrame(int index);

	int getNumberSavedFrames();

private:

	void assembleStats(int frameNumber, direction dir);
	statusTypes estimateNextVehicleData(Globals& g, int FrameNum);



	Vector <Snapshot> snaps;  // Keeps a history of all logged snapshots of vehicle as it moves.

	direction vehicleDirection;  // left, or right?   L2R v. R2L
	vehicleStatus vState; // entering, exiting, etc.
	statusTypes AmIOK;

	int nextY = 0; // Best estimate of next Y value for box around vehicle.  Note this is the upper value in open CV
	int nextHeight = 0; // Best estimate of next height value for box around vehicle
	int bestHeight = 0; // Best estimate of best height value for vehicle
	int bestWidth = 0; // Best estimate of width of vehicle, units are pixels.
	int estVelocity = 0; // Best estimate of current and near term pixel velocity of vehicle
	int bestVelocity = 0; // Best estimate of front bumper velocity, viewed in middle of ROI;

	bool deadReckonFB = false; // SHould front bumper be dead reckoned this cycle of frame differencing?
	bool deadReckonRB = false; // SHould rear bumper be dead reckoned this cycle of frame differencing?
	bool coasting = false;

	vector<double> FBFrame;  // USed to hold sequence of observed frame numbers in addsnaps for front bumper
	vector<double> FBPixel;  // Used to hold sequence of observed front bumpers (x coord) in addsnaps
	vector<double> RBFrame;  // USed to hold sequence of observed frame numbers in addsnaps for rear bumper
	vector<double> RBPixel;  // Used to hold sequence of observed rear bumpers (x coord) in addsnaps
	double FBSlope = 0.0;  // slope of front bumper pixel data over time
	double FBIntcpt = 0.0;  // x intercept of front bumper pixel data over time
	double RBSlope = 0.0; // slope of rear bumper pixel data over time
	double RBIntcpt = 0.0;  // x intercept of rear bumper pixel data over time
	double nextFrontBumper = 0.0; // Always contains the latest projected front bumper value
	double nextRearBumper = 0.0;  // Always contains the latest projected rear bumper value

	OverlapType overlapStatus = none;  //Initially assume vehicle overlaps no others;

	double LPFilterCoeff = 0.80;  // For filtering approximations of vehicle length (width)

// for stats

	int entryPixelIndex;
	int exitPixelIndex;
	int entryFrameNum;
	int exitFrameNum;

	int trackStartPixel = 0;
	int trackEndPixel= 0;
	int trackStartFrame = 0;
	int trackEndFrame = 0;

	int entryGap;
	int finalSpeed = -1;  // A final valid speed will be > 0;

// for hilites reel
	Mat lastFeed;
	vector<Mat> hiLiteFeeds;


};

