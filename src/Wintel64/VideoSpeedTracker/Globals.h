
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

#define MYLIB_CONSTANTS_H 1
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

#include <string>

enum direction { L2R, R2L, UNK };
enum vehicleStatus { entering, inMiddle, exiting, exited };
enum statusTypes { ImOK, deleteWithStats, lostTrack, negVelocity };
enum OverlapType { none, rearOnly, frontOnly, bothOverlap };
enum grabType { greedy, strict };

using namespace std;



class Globals
{
public:
	Globals();

	~Globals();

	bool readConfig();	

	const int pixelLeft = 0;				// This won't change.  It's used relative to AnalysisBoxLeft.
	int pixelRight = 1279;                  // This will change to be AnalysisBoxWidth.  Change should happen after AnalysisBoxLeft and Width are read in from config.

	// * * * * * * * * v v v v * * * * * * * * R e a d i n g   o f   C o n f i g    f i l e   s h o u l d    o v e r r i d e   t h e s e * * v v v v * * * * * * //

	string dataPathPrefix = "g:\\locustdata";	// path to data directories, IPCam, Stats, etc.  Don't use double backslash in config file!.
	string L2RDirection = "SE";					// Compass Direction L2R vehicles are heading
	string R2LDirection = "NW";					// Compass Direction R2L vehicles are heading
	int obstruction[2];				// left / right x coords bounding vertical obstructions in foreground.Max 2. Relative to AnalysisBoxLeft. ...pixels
	int AnalysisBoxLeft = 10;		// How much to crop off left side of full frame for analysis box. Want to keep this small(zero!) if possible
	int AnalysisBoxTop = 220;		// How much to crop off top of full frame to get rid of tree tops, across - the - street houses, etc.
	int AnalysisBoxWidth = 1269;	// Width of Analysis Box relative to AnalysisBoxLeft.Their sum must be <= 1279. ...pixels
	int AnalysisBoxHeight = 190;	// measured down from Analysis Box Top; determines height of Analysis Box.  (Top + Height) <= 719.
	int speedLineLeft = 350;		// x coord of left white line for speed measuring box, realtive to AnalysisBoxLeft
	int speedLineRight = 909;		// x coord of right white line for speed measuring box, realtive to AnalysisBoxLeft
	int maxL2RDistOnEntry = 75;		//  Maximum trackable speed, delta pixels per two frames.  Represents about 70MPH;
	int maxR2LDistOnEntry = 75;		//  Maximum trackable speed, delta pixels per two frames.  Represents about 70MPH;
	int entryLookBack = 250;		//  Use this to force looking back as vehicle enters Analysis Box.
	int obstruction_extent = 30;	// Even after a vehicle has passed an obstruction it needs space to show up as a blob past the obstruction in differencing operation.
	int largeVehicleArea = 109;		// After dividing a computed area by 300.  This value tends to separate buses and UPS trucks from large pickups.
	int CalibrationFramesL2R = 35;	// Measured three times on 14 Jan 2016 for Locust Avenue, Charlottesville, VA
	int CalibrationFramesR2L = 40;	// Measured three on 14 Jan 2016 for Locust Avenue, Charlottesville, VA
	int SENSITIVITY_VALUE = 30;		// Sensitivity value for the OpenCV absdiff funtion. Change with care.
	int	BLUR_SIZE = 20;				// To smooth the intensity image output from absdiff() function. Change with care.
	int	SLOP = 15;					// Margin of error when testing for vehicle overlap... pixels.
	int R2LStreetY = 122;			// Hubcap line for R2L vehicles on flat street.Orange. Locust Ave.  Relative to AnalysisBoxTop...pixels
	int L2RStreetY = 158;			// Hubcap line for L2R vehicles on flat street.Purple. Locust Ave. Relative to AnalysisBoxTop...pixels
	int nextHeight = 85;			// Initial best guess for height of entering vehicles...pixels.

private:


};
