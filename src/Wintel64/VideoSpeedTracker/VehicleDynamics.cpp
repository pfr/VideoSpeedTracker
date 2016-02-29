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



#include "VehicleDynamics.h"
#include "Globals.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>
#include "Projection.h"
#include "Snapshot.h"


VehicleDynamics::VehicleDynamics()
{
	VehicleDynamics::estVelocity = -1;
}

VehicleDynamics::VehicleDynamics(direction dir)
{
	VehicleDynamics::vehicleDirection= dir;
	VehicleDynamics::estVelocity = -1;
}

VehicleDynamics::~VehicleDynamics()
{
}


int VehicleDynamics::getTrackStartPixel(){
	return trackStartPixel;
}
int VehicleDynamics::getTrackEndPixel(){
	return trackEndPixel;
}
int VehicleDynamics::getTrackStartFrame(){
	return trackStartFrame;
}
int VehicleDynamics::getTrackEndFrame(){
	return trackEndFrame;
}


int VehicleDynamics::getArea(){
	// Return a scaled value for ease of analysis (divide by 300)
	return (bestHeight * bestWidth) / 300;
}

statusTypes VehicleDynamics::getAmIOK(){
	return AmIOK;
}

double VehicleDynamics::getFBSlope(){
	return FBSlope; 
}

double VehicleDynamics::getFBIntercept(){
	return FBIntcpt;
}

double VehicleDynamics::getRBSlope(){
	return RBSlope;
}

double VehicleDynamics::getRBIntercept(){
	return RBIntcpt;
}

double VehicleDynamics::getNextFrontBumper(){
	return nextFrontBumper;
}
double VehicleDynamics::getNextRearBumper(){
	return nextRearBumper;
}

void VehicleDynamics::setOverlapStatus(OverlapType inOverlapStatus){
	overlapStatus = inOverlapStatus;

}

OverlapType VehicleDynamics::getOverlapStatus(){
	return overlapStatus;
}

int VehicleDynamics::getFinalSpeed(){
	return finalSpeed;
}


void VehicleDynamics::holdFrame(Mat inMat){
	lastFeed = inMat;
}

void VehicleDynamics::saveFrame(Mat inMat){
	hiLiteFeeds.push_back(inMat);
}

Mat VehicleDynamics::getHeldFrame(){
	return lastFeed;
}

Mat VehicleDynamics::getSavedFrame(int index){
	if (hiLiteFeeds.size() > index)
		return hiLiteFeeds[index];
	else return Mat();
}

int VehicleDynamics::getNumberSavedFrames(){
	return hiLiteFeeds.size();
}

// Linear least squares method for fitting line through a set of x,y pairs.  Return slope and intercept.
vector<double> getLinearFit(const std::vector<double>& x, const std::vector<double>& y) {
	vector<double> res;
	const auto n = x.size();
	const auto s_x = std::accumulate(x.begin(), x.end(), 0.0);
	const auto s_y = std::accumulate(y.begin(), y.end(), 0.0);
	const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
	const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
	const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
	res.push_back(a);                      // return slope
	res.push_back((s_y - a * s_x) / n);    // followed by intercept.
	return res;
}



//Private function
void VehicleDynamics::assembleStats(int frameNumber, direction dir){
	entryFrameNum = snaps.front().getFrameNum();
	exitFrameNum = frameNumber - 2; // -2 because the car entered the exiting region in previous frame pair.
	if (dir == L2R){

		entryPixelIndex = snaps.front().getRect().x + snaps.front().getRect().width;
		exitPixelIndex = snaps.back().getRect().x + snaps.back().getRect().width;
	}
	else{
		entryPixelIndex = snaps.front().getRect().x;
		exitPixelIndex = snaps.back().getRect().x;
	}
}


void VehicleDynamics::addSnapshot(Snapshot inShot){
//	cout << "Box coming in to addSnapShot: " << inShot.getRect().x << " " << inShot.getRect().y << " " << inShot.getRect().width << " " << inShot.getRect().height << endl;
	VehicleDynamics::snaps.push_back(inShot);

	if (snaps.size() == 1){
		vState = entering;
	}
	return;
}


bool validGap(int gap){
//	quality measure to determine if tracking worked well enough to report vehicle speed.
	return abs(gap) <= 40;
}


void VehicleDynamics::markInvalidSpeed(){
	trackEndPixel = -1;
}

int VehicleDynamics::computeFinalSpeed(Globals& g, direction dir, int trackStartFrame, int trackEndFrame, int trackStartPixel, int trackEndPixel, int entryGap, int endGap, double estVel){
	
	// The entry and end gaps may provide useful infomration for minor speed assessment corrections.  Not using them here yet...
	int halfSpeed = int(estVel / 2.0);
	switch (dir){
	case L2R:
		// fine tune final frame marker used for estimating speed
		if ((trackStartPixel - g.speedLineLeft) > halfSpeed && (trackEndPixel - g.speedLineRight) < halfSpeed)  trackEndFrame++;  // went over start late, and left early
		else if ((trackStartPixel - g.speedLineLeft) < halfSpeed && (trackEndPixel - g.speedLineRight) > halfSpeed)  trackEndFrame--; // went over start early, and left late
		return int(((double(g.CalibrationFramesL2R) / double(trackEndFrame - trackStartFrame)) * 25.0) + 0.4999);
	case R2L:
		// fine tune final frame markers used for estimating speed
		if ((g.speedLineRight - trackStartPixel) > halfSpeed && (g.speedLineLeft - trackEndPixel) < halfSpeed)  trackEndFrame++;  // went over start as late as possible and stayed late
		else if ((g.speedLineRight - trackStartPixel) < halfSpeed && (g.speedLineLeft - trackEndPixel) > halfSpeed)  trackEndFrame--; // went over start early, and left late
		return int(((double(g.CalibrationFramesR2L) / double(trackEndFrame - trackStartFrame)) * 25.0) + 0.4999);
	case UNK:
		return -1;
	}
	return -1;
}






// *****************************************************************************************************
// Private function that uses linear regression to derive values for front and rear bumpers, and estVelocity.
// Simpler algorithms are used to predict bestHeight and nextY. Uses /snaps/ and /projections/ histories.
// Function also returns one of four status types: ImOK, deleteWithStats, lostTrack, negVelocity
// *****************************************************************************************************
statusTypes VehicleDynamics::estimateNextVehicleData(Globals& g, int frameNum){

	Rect lastObservedBox = snaps.back().getRect();
	double prevNextFrontBumper = nextFrontBumper; // Get last prediction for front bumper
	double prevNextRearBumper = nextRearBumper;  // Get last prediction for rear bumper
	int prevEstVelocity = estVelocity;

//  * * * * * * * * * * * * * * * * * * * * * * M a i n t a i n   s n a p s h o t   q u a l i t y * * * * * * * * * * * * * * * * * * * * * * * *

	int snapsDiff = ((frameNum - snaps.back().getFrameNum()) / 2);  // Snaps don't always occur for a tracked vehicle, so we need to gauge how long since last.

	if (snapsDiff > 2 || ((snapsDiff == 2) && coasting)){ // It's been too long: apparently lost track.  "3" is chosen a bit arbitrarily.
		assembleStats(frameNum, vehicleDirection);
		return lostTrack;
	}

	if (snapsDiff == 2) {
		if (vehicleDirection == L2R)
			addSnapshot(Snapshot(Rect(int(prevNextRearBumper+0.5), nextY, int(prevNextFrontBumper - prevNextRearBumper + 0.5), bestHeight), frameNum - 2));
		else addSnapshot(Snapshot(Rect(int(prevNextFrontBumper + 0.5), nextY, int(prevNextRearBumper - prevNextFrontBumper + 0.5), bestHeight), frameNum - 2));
		coasting = true;
	}
	else coasting = false; // Executed if snapsDiff == 1;

//  * * * * * * * * * * * * * * * * * * * * * * O n e   s n a p s h o t   c a s e * * * * * * * * * * * * * * * * * * * * * * * *

	if (snaps.size() == 1){
		// No previous projections, and only one snapshot so far, must be in entering state.

		FBFrame.push_back(double(snaps.back().getFrameNum()));  // build FBFrame vector, allowing for skipped frame pairs.

		switch (vehicleDirection){
		case L2R:  // L2R >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			nextRearBumper = max(g.pixelLeft, ((lastObservedBox.x + lastObservedBox.width) - g.entryLookBack));
			nextFrontBumper = double(lastObservedBox.x + lastObservedBox.width + g.maxL2RDistOnEntry);
			FBPixel.push_back(double(lastObservedBox.x + lastObservedBox.width));
			nextHeight = g.nextHeight;
			bestHeight = nextHeight;
			break;
		case R2L:  //R2L <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			nextRearBumper = min(g.pixelRight, lastObservedBox.x + g.entryLookBack);
			nextFrontBumper = double(lastObservedBox.x - g.maxR2LDistOnEntry);
			FBPixel.push_back(double(lastObservedBox.x));
			nextHeight = g.nextHeight;
			bestHeight = nextHeight;
			break;
		case UNK: // UNK ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ?
			break;
		}
			nextY = 60;
			estVelocity = 10;  // Ten is an estimate only, on the conservative side, generally placing rear bumper further back than actual, when it is used in next block.
			return ImOK;
	}

// * * * * * * * * * * * * * * * * * * * * * * *
//  * * * * * * * * * * * * * * * * * * * * * * T w o / t h r e e   s n a p s h o t s   c a s e * * * * * * * * * * * * * * * * * * * * * * * *
	// At least two snapshots and one projection are available. Not enough data to do linear regression on FB yet -- Need four data points.
	// Dead reckon projection of entering vehicle FB for worst case.  Rear bumper may be in view, and vehicle may be occluded.

	Rect nextToLastBox = snaps[snaps.size() - 2].getRect();

	if (snaps.size() <= 3){ // Still trying to get track on front bumper.  Assumption here is that vehicle is still entering.

		FBFrame.push_back(double(snaps.back().getFrameNum()));  // build FBFrame vector.

		switch (vehicleDirection){
		case L2R:  // L2R >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			if (overlapStatus == none || overlapStatus == rearOnly){
				double beliefFactor = double(snaps.size() - 1) / double(snaps.size());
				double motionDelta = max(double(lastObservedBox.x + lastObservedBox.width) - double(nextToLastBox.x + nextToLastBox.width), 10.0);
				nextFrontBumper = double(lastObservedBox.x + lastObservedBox.width) + (beliefFactor * motionDelta)
					+ ((1.0 - beliefFactor) *  g.maxL2RDistOnEntry);  // Believe part of how much moved last time and part of aggressive look out front.
				FBPixel.push_back(double(lastObservedBox.x + lastObservedBox.width));
				if (trackStartPixel == 0 && (nextFrontBumper >= g.speedLineLeft)){
					entryGap = int(prevNextFrontBumper) - (lastObservedBox.x + lastObservedBox.width);
					trackStartPixel = int(nextFrontBumper);  // Start tracking speed
					trackStartFrame = frameNum;
				}
			}
			else { // overlap status is front only or both.  Unfortunately, a good estimate of FB velocity has not been computed yet.
				//  Therefore it's proabably best to drop for now.  May still be seen as an entering vehicle once it reemerges.
				return lostTrack;
			}

			if (lastObservedBox.x < 45 || int(nextFrontBumper) < g.entryLookBack  // A rear bumper less than 45 pixels away from left edge is still at left edge.  45 is a tuning parameter.  
				                       || (overlapStatus == rearOnly || overlapStatus == bothOverlap)) // vehicle's rear bumper not determined yet; could still be zero
				nextRearBumper = g.pixelLeft;
			else { // Rear bumper has left the left edge;  Start collecting data for Linear regression over rear bumper
				nextRearBumper = double(lastObservedBox.x + estVelocity); // Make nextRearBumper take on value of last observed rear bumper + exp change.
				RBFrame.push_back(double(snaps.back().getFrameNum()));
				RBPixel.push_back(double(lastObservedBox.x));
			}
			nextHeight = g.nextHeight;
			break;

		case R2L:  //R2L <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			if (overlapStatus == none || overlapStatus == rearOnly){
				double beliefFactor = double(snaps.size() - 1) / double(snaps.size());
				double motionDelta = max(double(nextToLastBox.x - lastObservedBox.x), 10.0);
				nextFrontBumper = (double)lastObservedBox.x - (beliefFactor * motionDelta )
					- ((1.0 - beliefFactor) *  g.maxR2LDistOnEntry);  // Can keep using FB (x) because rear bumper projection is parked at pixelRight;
				FBPixel.push_back(double(lastObservedBox.x));
				if (trackStartPixel == 0 && (nextFrontBumper <= g.speedLineRight)){
					entryGap = int(prevNextFrontBumper) - lastObservedBox.x;
					trackStartPixel = int(nextFrontBumper);  // Start tracking speed
					trackStartFrame = frameNum;
				}
			}
			else{ // overlap status is front only or both.  Since the vehicle is entering, a good estimate of its velocity has not been computed yet.
				//  Therefore it's proabably best to drop it for now.  It may still be seen as an entering vehicle once it reemerges.
				return lostTrack;
			}

			if ((lastObservedBox.x + lastObservedBox.width) > (g.pixelRight - 45) || (g.pixelRight - int(nextFrontBumper)) < g.entryLookBack   //  See note above about use of 45.
				|| (overlapStatus == rearOnly || overlapStatus == bothOverlap))
				nextRearBumper = g.pixelRight;
			else { // This will force a transition to vState = inMiddle
				nextRearBumper = lastObservedBox.x + lastObservedBox.width - estVelocity;
				// Start collecting data for Linear regression over rear bumper
				RBFrame.push_back(double(snaps.back().getFrameNum()));
				RBPixel.push_back(double(lastObservedBox.x + lastObservedBox.width));
			}
			nextHeight = g.nextHeight;
			break;

		case UNK: // UNK ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ?
			break;

		}

		estVelocity = abs((lastObservedBox.x + lastObservedBox.width) - (nextToLastBox.x + nextToLastBox.width)); // Could be moving backwards!  In normal case, delta between
																												// two front bumpers in sequence is best estimate.
		if ((vehicleDirection == L2R && (nextRearBumper >= nextFrontBumper)) || (vehicleDirection == R2L && (nextRearBumper <= nextFrontBumper))){
			assembleStats(frameNum, vehicleDirection);
			return lostTrack;
		}

		return ImOK;
	}

// * * * * * * * * * * * * * * * * * * * * * 
// * * * * * * * * * * * * * * * * * * * * *     F o u r    o r    m o r e    s n a p s h o t s     * * * * * * * * * * * * * * * *
// * * * * * * * * * * * * * * * * * * * * * 




	// IF not exiting, exited, or DR'ing FB, save latest snapshot.  Look for obstructions, veh overlap and apparent backwards motion.
	// Compute slope of the front bumper locations, to see if the vehicle is moving backwards.
	// If vehicle is moving backwards, remove it from further consideration.  Otherwise, use estimate of FBSlope for FB projections (rather than maxDistOnEntry).

	vector<double> FBSlopeInt;  // temporary container for FB slope and intercept coming back from call to getLinearFit()

	if (vState == entering || vState == inMiddle){
		if (deadReckonFB){
			if (vehicleDirection == L2R) nextFrontBumper = prevNextFrontBumper + estVelocity;   // DR'ing FB, no slope analysis needed;
			else nextFrontBumper = prevNextFrontBumper - estVelocity;
		}
		else {
			if (FBFrame.size() >= 8){
				FBFrame.erase(FBFrame.begin());  // Experimental: do the linear regression over the last n FB data points...piecewise.  
				FBPixel.erase(FBPixel.begin());  //              These deletions accommodate changing camera lens disotrtion.
			}
			FBFrame.push_back(double(snaps.back().getFrameNum())); // Assert:  FBframe.size() == snaps.size()

			// Deal with obstructions and apparent backward motion of front bumper raw data

			switch (vehicleDirection){
			case L2R:
				// Check for obstructions or apparent backward motion of front bumper raw data
				if (((int(prevNextFrontBumper) >= g.obstruction[0]) && (int(prevNextFrontBumper) <= (g.obstruction[1] + g.obstruction_extent)))
					|| ((lastObservedBox.x + lastObservedBox.width) < (nextToLastBox.x + nextToLastBox.width + int(estVelocity / 2.0))) // Unacceptable backwards motion of actual data
					|| (overlapStatus == frontOnly || overlapStatus == bothOverlap)){
					FBPixel.push_back(prevNextFrontBumper);  // 
					//					cout << " * * * * * * * * * * * * * * L2R front bumper moved backwards or it was occluded." << endl;
				}
				else FBPixel.push_back(double(lastObservedBox.x + lastObservedBox.width));
				break;
			case R2L:   //  R2L <<<<<
				if (((int(prevNextFrontBumper) >= (g.obstruction[0] - g.obstruction_extent)) && (int(prevNextFrontBumper) <= g.obstruction[1]))
					|| (lastObservedBox.x > (nextToLastBox.x - int(estVelocity / 2.0))) // Unacceptable backwards motion of actual data
					|| (overlapStatus == frontOnly || overlapStatus == bothOverlap)){
					FBPixel.push_back(double(prevNextFrontBumper));
					//					cout << " * * * * * * * * * * * * * * R2L front bumper moved backwards or it was occluded." << endl;
				}
				else FBPixel.push_back(double(lastObservedBox.x));
				break;
			case UNK:
				break;
			};

			FBSlopeInt = getLinearFit(FBFrame, FBPixel);  // Get the slope and intercept of selected number of past observed frontbumpers.
			FBSlope = FBSlopeInt[0];  // Get slope;
			FBIntcpt = FBSlopeInt[1]; // Get intercept;

			if ((vehicleDirection == L2R && (FBSlope < 0)) || (vehicleDirection == R2L && (FBSlope > 0))){
				assembleStats(frameNum, vehicleDirection);
				return negVelocity;
			}

			//****
			nextFrontBumper = FBSlope * frameNum + FBIntcpt; // Big deal.  Using linear regression to project FB from accumulated snapshots.  Y = mx + b.
			// Note: if transitioning to exiting, FB may have just moved outside of analysis box (e.g. < pixLeft or > pixRight)
			//****

		} // Section where DR'ing of FB is not happening.



		if (nextFrontBumper < g.pixelLeft) nextFrontBumper = g.pixelLeft;           // Stay in
		else if (nextFrontBumper > g.pixelRight) nextFrontBumper = g.pixelRight;    // bounds


//   * * * * * * * * * * * * * * * * * * * * * * * *  Has vehicle just crossed one of the speed measuring box white lines? * * * * * * * * * * * * * * *

		switch (vehicleDirection){ // Start / stop  tracking speed???
		case L2R:
			if ((trackStartPixel == 0) && (nextFrontBumper >= g.speedLineLeft)){
				entryGap = int(prevNextFrontBumper) - (lastObservedBox.x + lastObservedBox.width);
				//				cout << "Entry gap: " << entryGap << endl;
				trackStartPixel = int(nextFrontBumper); // Start tracking L2R speed
				trackStartFrame = frameNum;
			}
			if ((trackEndPixel == 0) && (nextFrontBumper > g.speedLineRight)){
				int endGap = (int(prevNextFrontBumper) - (lastObservedBox.x + lastObservedBox.width));
				if (validGap(entryGap - endGap)){
					trackEndPixel = int(nextFrontBumper); // End tracking L2R speed
					trackEndFrame = frameNum;
					finalSpeed = computeFinalSpeed(g, L2R, trackStartFrame, trackEndFrame, trackStartPixel, trackEndPixel, entryGap, endGap, estVelocity);
					cout << "<" << frameNum << ">    > > > > > Speed is : " << finalSpeed << endl;
					deadReckonFB = true;  // Once speed measurement is done, just dead reckon vehicle out of the picture.
				}
				else {
					cout << "<" << frameNum << ">   X X X X > INVALID SPEED MEASUREMENT (" << entryGap - endGap << ")" << endl;
					trackEndPixel = -1;
				}
			}
			break;

		case R2L: // handle R2L <<<<<<<<<<<<<<<<<<<<<<<<<<
			if (trackStartPixel == 0 && (nextFrontBumper <= g.speedLineRight)){
				entryGap = int(prevNextFrontBumper) - lastObservedBox.x;
				//				cout << "Entry gap: " << entryGap << endl;
				trackStartPixel = int(nextFrontBumper);  // Start tracking R2L speed
				trackStartFrame = frameNum;
			}
			if ((trackEndPixel == 0) && (nextFrontBumper < g.speedLineLeft)){
				int endGap = int(prevNextFrontBumper) - lastObservedBox.x;
				if (validGap(entryGap - endGap)){
					trackEndPixel = int(nextFrontBumper); // End tracking R2L speed
					trackEndFrame = frameNum;
					finalSpeed = computeFinalSpeed(g, R2L, trackStartFrame, trackEndFrame, trackStartPixel, trackEndPixel, entryGap, endGap, estVelocity);
					cout << "<" << frameNum << ">   < < < < < Speed is : " << finalSpeed << endl;
					deadReckonFB = true;    // Once speed measurement is done, just dead reckon vehicle out of the picture.
				}
				else {
					cout << "<" << frameNum << ">   < X X X X INVALID SPEED MEASUREMENT (" << entryGap - endGap << ")" << endl;
					trackEndPixel = -1;
				}
			}
			break;
		case UNK:
			break;
		} // Switch


		if (vState == entering){
			switch (vehicleDirection){  // L2R >>>>>>>>>>>>>>>>>>>>>>>
			case L2R:
				if (lastObservedBox.x < 45 || int(nextFrontBumper) < g.entryLookBack   // A rear bumper less than 45 pixels away from left edge is still at left edge.  45 is a tuning parameter.  
					|| (overlapStatus == rearOnly || overlapStatus == bothOverlap)) // vehicle's rear bumper not determined yet; could still be zero
					nextRearBumper = g.pixelLeft;
				else { // This will force a transition to vState = inMiddle
					nextRearBumper = double(lastObservedBox.x + estVelocity); // Make nextRearBumper take on value of last observed rear bumper + exp change.  Just left entering state.
					// Start collecting data for Linear regression over rear bumper
					RBFrame.push_back(double(snaps.back().getFrameNum()));
					RBPixel.push_back(double(lastObservedBox.x));
				}
				break;
			case R2L:  // R2L  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				if ((lastObservedBox.x + lastObservedBox.width) > (g.pixelRight - 45) || (g.pixelRight - int(nextFrontBumper)) < g.entryLookBack  // See note about 45, just above.
					|| (overlapStatus == rearOnly || overlapStatus == bothOverlap))
					nextRearBumper = g.pixelRight;
				else { // This will force a transition to vState = inMiddle
					nextRearBumper = lastObservedBox.x + lastObservedBox.width - estVelocity;
					// Start collecting data for Linear regression over rear bumper
					RBFrame.push_back(double(snaps.back().getFrameNum()));
					RBPixel.push_back(double(lastObservedBox.x + lastObservedBox.width));
				}
				break;
			case UNK:
				break;
			}
		} // end handling entering


// vState == inMiddle; 
		else { // It is known that rear bumper has moved inside ROI.  Front bumper may be about to move out of ROI
			if (deadReckonRB)   // Dead reckon rear bumper after confidence about width is high.
				if (vehicleDirection == L2R)
					nextRearBumper = prevNextRearBumper + estVelocity;
				else nextRearBumper = prevNextRearBumper - estVelocity;

			else {  // Keep computing rear bumper from linear regression

				RBFrame.push_back(double(snaps.back().getFrameNum()));
	
	// Determine if last prediction for rear bumper is occluded or bumper has moved backwards;  if so replace last RBPixel with previous projected value.
				switch (vehicleDirection){    // >>>>>>>>>>>>>>>>>>>>> L2R  >>>>>>>>>>>>>>>>>>>>>>>>>>
				case L2R:
					if (((int(prevNextRearBumper) >= (g.obstruction[0] - g.obstruction_extent)) && (int(prevNextRearBumper) <= g.obstruction[1]))
						|| (lastObservedBox.x < nextToLastBox.x) || (overlapStatus == rearOnly || overlapStatus == bothOverlap)) {
						RBPixel.push_back(prevNextRearBumper);  // 
					}
					else RBPixel.push_back(double(snaps.back().getRect().x));
					break;
				case R2L:  // <<<<<<<<<<<<<<< R2L <<<<<<<<<<<<<<<<<<<<<<<
					if (((int(prevNextRearBumper) >= g.obstruction[0]) && (int(prevNextRearBumper) <= (g.obstruction[1] + g.obstruction_extent)))
							 || ((lastObservedBox.x + lastObservedBox.width) >(nextToLastBox.x + nextToLastBox.width)) || (overlapStatus == rearOnly || overlapStatus == bothOverlap)) {
						RBPixel.push_back(prevNextRearBumper);  // 
					}
					else RBPixel.push_back(double(snaps.back().getRect().x + snaps.back().getRect().width));
					break;
				case UNK:
					break;
				}

// If RBFrame.size() <= 5 then compute a value for nextRearBumper and exit parent if statement

				if (RBPixel.size() <= 5){
					if (vehicleDirection == L2R) nextRearBumper += estVelocity;
					else nextRearBumper -= estVelocity;
				}
				else {
  // Check for excess of entries (to keep linear regression piecewise)
					if (RBFrame.size() >= 8){
						RBFrame.erase(RBFrame.begin());  // Experimental: do the linear regression over the last eight RB data points...piecewise.  
						RBPixel.erase(RBPixel.begin());  //              These deletions accommodate changing camera pixel density.
					}

					// Fit a curve through RBPixel points to determine a rear bumper pixel.
					vector<double> RBSlopeInt;  // temporary container for slope and intercept coming back from call to getLinearFit()
					RBSlopeInt = getLinearFit(RBFrame, RBPixel);  // Get the slope and intercept of all past observed rear bumpers.
					RBSlope = RBSlopeInt[0];  // Get slope;
					RBIntcpt = RBSlopeInt[1]; // Get intercept;

// *****
					nextRearBumper = RBSlope * frameNum + RBIntcpt; // Big deal.  Using linear regression to project RB from accumulated snapshots.  Y = mx + b.
// *****
				}

			}

// Don't let rear bumper move backwards.  OTOH, don't move it forward too aggressively, since discovery of length of vehicle may still be occurring.
			if (vehicleDirection == L2R){
				if (nextRearBumper < prevNextRearBumper) nextRearBumper = prevNextRearBumper + estVelocity / 2;
				if (nextRearBumper > g.speedLineRight) deadReckonRB = true;
			}
			else {
				if (nextRearBumper > prevNextRearBumper) nextRearBumper = prevNextRearBumper - estVelocity / 2;
				if (nextRearBumper < g.speedLineLeft) deadReckonRB = true;
			}


// Note: if transitioning to exiting, RB may have just moved outside of Analysis box (e.g. < pixLeft or > pixRight)

			if (nextRearBumper < g.pixelLeft) nextRearBumper = g.pixelLeft;
			else if (nextRearBumper > g.pixelRight) nextRearBumper = g.pixelRight;

// Has rear bumper projection caught up to front?  Bail if so.

			if ((vehicleDirection == L2R && (nextRearBumper >= nextFrontBumper)) || (vehicleDirection == R2L && (nextRearBumper <= nextFrontBumper))){
				assembleStats(frameNum, vehicleDirection);
				return lostTrack;
			}


// Get best snapshot of cross section area for stats (taken when center of vehicle in center of Analysis box)
			if (abs(((nextFrontBumper + nextRearBumper) / 2) - ((g.pixelRight - g.pixelLeft) / 2)) <= 70){
				bestHeight = lastObservedBox.height;
				bestWidth = lastObservedBox.width;
				bestVelocity = estVelocity;

			}
		}
		
		estVelocity = (abs(int(prevNextFrontBumper - nextFrontBumper)) + prevEstVelocity) / 2; // A little smoothing
		
		if ((vehicleDirection == L2R && (nextRearBumper >= nextFrontBumper)) || (vehicleDirection == R2L && (nextRearBumper <= nextFrontBumper))){
			assembleStats(frameNum, vehicleDirection);
			return lostTrack;
		}

	} // end entering or middle


	else { // by default, vState == exiting;  dead reckon outta here.
		switch (vehicleDirection){    // >>>>>>>>>>>>>>>>>>>>> L2R  >>>>>>>>>>>>>>>>>>>>>>>>>>
		case L2R:
			nextRearBumper = min(int(prevNextRearBumper + bestVelocity), g.pixelRight);  // Note estVelocity is not changing once vState == exiting is reached. 
			nextFrontBumper = double(g.pixelRight);
			break;
		case R2L: //  R2L <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			nextRearBumper = max(int(prevNextRearBumper - bestVelocity), g.pixelLeft);   // Note estVelocity is not changing once vState == exiting is reached. 
			nextFrontBumper = double(g.pixelLeft);
			break;
		case UNK:
			break;
		}
	}

// Done for all vStates...
	nextY = (lastObservedBox.y + nextY) / 2;
	nextHeight = (lastObservedBox.height + bestHeight) / 2;
	return ImOK;
	
} // end <estimateNextVehicleData()>



// *****************************************************************************************************
// Public function that returns best projection possible, plus state, for vehicle in next frame pair.
// *****************************************************************************************************

Projection VehicleDynamics::getBestProjection(Globals& g, int frameNum){

// Possible vStates:  entering, inMiddle, exiting, exited

	if (snaps.size() == 0){
		cout << "In vehicleDynamics, trying to get a projection when none exists! Prog should exit here, but for now keeps going" << endl;
		vState = entering;
		return Projection(Rect(0, 0, 0, 0), vState, 0, frameNum);
	}

	AmIOK = estimateNextVehicleData(g, frameNum);

	if (AmIOK == lostTrack || AmIOK == negVelocity){
		return Projection(Rect(0, 0, 0, 0), vState, 0, frameNum);
	}

	if (snaps.size() == 1){
		vState = entering; 
		if (vehicleDirection == L2R)
			return Projection(Rect(int(nextRearBumper), nextY, int(nextFrontBumper - nextRearBumper), nextHeight), vState, estVelocity, frameNum);
		else 
			return Projection(Rect(int(nextFrontBumper), nextY, int(nextRearBumper - nextFrontBumper), nextHeight), vState, estVelocity, frameNum);
	}

// snapCount >= 2

// Vehicle direction is L2R >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	if (vehicleDirection == L2R){    // vehicleDirection == L2R

		switch (vState){
		case entering:
			if (int(nextRearBumper) > g.pixelLeft) vState = inMiddle;  // rear bumper has appeared.
			break;
		case inMiddle:
			if (int(nextFrontBumper) >= g.pixelRight){ // front bumper crossing far edge of analysis box
				vState = exiting;
				assembleStats(frameNum, L2R);
			}
			break;
		case exiting:
			if ((int(nextRearBumper) >= g.pixelRight) ) // L2R vehicle has exited right; convey that to caller via vState being set to #exited#
				vState = exited;
			break;
		default:
			cout << "Never should have gotten here L2R in vehicle projection. vState = " << vState << endl;
			return Projection(Rect(0, 0, 0, 0), exited, 0, frameNum);

		} // switch

		return Projection(Rect(int(nextRearBumper), nextY, int(nextFrontBumper - nextRearBumper), nextHeight), vState, estVelocity, frameNum);
	}
// Vehicle direction is R2L <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	else  {  // vehicleDirection == R2L

		switch (vState){
		case entering:
			if (int(nextRearBumper) < g.pixelRight) vState = inMiddle; // rear bumper has appeared.
			break;
		case inMiddle:
			if (int(nextFrontBumper) <= g.pixelLeft){ /// front bumper crossing far edge of analysis box
				vState = exiting;
				assembleStats(frameNum, R2L);
			}
			break;
		case exiting:
			if ((int(nextRearBumper) <= g.pixelLeft) ){ // R2L vehicle has exited left; convey that to caller via vState being set to #exited#
				vState = exited;
			}
			break;
		default:
			cout << "Never should have gotten here R2L in vehicle projection. vState = " << vState << endl;
			return Projection(Rect(0, 0, 0, 0), exited, 0, frameNum);

		}

		return Projection(Rect(int(nextFrontBumper), nextY, int(nextRearBumper - nextFrontBumper), nextHeight), vState, estVelocity, frameNum);
	}
		
};
