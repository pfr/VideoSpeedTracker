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


#include "Globals.h"
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
// #include <string>

using namespace std;

Globals::Globals()
{
}


Globals::~Globals()
{
}

string inLine, lhs, rhs;


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
	size_t last = toBeTrimmed.length() - 1;
	while (toBeTrimmed.substr(last, 1) == " ") last--;
	return toBeTrimmed.substr(first, (last-first)+1);
}



bool Globals::readConfig(){

// Items in config file VST.cfg must conform WRT order and spelling of LHS items, as follows:
//  VST.cfg must use syntax:  <LHS> = <RHS> # 
//                                            ^^^^^ Anything can follow the #

// First look for a config file that identifies different neighborhoods that would each have their own configuration values.
// This file should be named neighborhoods.cfg
// If there isn't a neighborhood config file, then assume there's only one neighborhood with its config values defined in a file named VST.cfg.
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
			chosenNeighborhood = neighborhoods.at(index-1);
			cout << endl;
		}
		cout << "Chosen neighborhood is: " << chosenNeighborhood << endl;
		dataPathPrefix = dataPathPrefix + "\\" + chosenNeighborhood + "Data";
		hoodsIn.close();
	}
	else	
		cout << "No neighborhoods.cfg file.  Looking for VST.cfg next." << endl;




	string lhsString[23] = {
		"dataPathPrefix",
		"L2RDirection",
		"R2LDirection",
		"obstruction", 
		"AnalysisBoxLeft",
		"AnalysisBoxTop",
		"AnalysisBoxWidth",
		"AnalysisBoxHeight",
		"speedLineLeft",
		"speedLineRight",
		"maxL2RDistOnEntry",
		"maxR2LDistOnEntry",
		"entryLookBack",
		"obstruction_extent",
		"largeVehicleArea",
		"CalibrationFramesL2R",
		"CalibrationFramesR2L",
		"SENSITIVITY_VALUE",
		"BLUR_SIZE",
		"SLOP",
		"R2LStreetY",
		"L2RStreetY",
		"nextHeight"
	};
	ifstream configIn;
	if (chosenNeighborhood == ""){
//		cout << "Trying to open file VST.cfg" << endl;
		configIn.open("VST.cfg");
		if (!configIn.good()){
			cout << "Can't open VST.cfg." << endl;
			return false;
		}
	}
	else {
//		cout << "Trying to open file " << dataPathPrefix << "\\" << chosenNeighborhood << "Data.cfg" << endl;
		configIn.open(dataPathPrefix + "\\" + chosenNeighborhood + "Data.cfg");
		if (!configIn.good()){
			cout << "Can't open " << chosenNeighborhood << "Data.cfg" << endl;
			return false;
		}
	}

	size_t place, place2;
	lineNo = 0;
	while (getline(configIn, inLine)){
		;
		if (!configIn){
			cout << "Unexpected error in config file." << endl;
			return false;
		}
		while (inLine.back() == '\r')
			inLine.pop_back();
		if (inLine.empty())
			continue;  // blank line
		if (getSides(inLine)){
			lhs = trim(lhs);
			rhs = trim(rhs);
			if (lhs != lhsString[lineNo]){
				cout << "Just read LHS doesn't match anything: <" << lhs << ">.  Aborting" << endl;
				return false;
			}
			switch (lineNo){
			case 0:             // dataPathPrefix
				if (chosenNeighborhood == "")
					dataPathPrefix = rhs;
				cout << "dataPathPrefix = " << dataPathPrefix << endl;
				break;
			case 1:             // L2RDirection
				L2RDirection = rhs;
				cout << "L2RDirection = " << L2RDirection << endl;
				break;
			case 2:             // R2LDirection
				R2LDirection = rhs;
				cout << "R2LDirection = " << R2LDirection << endl;
				break;
			case 3:             // obstruction  (If no foreground obstructions in scene, just make first value >= second value)
				place = rhs.find(',');
				place2 = rhs.find(']');
				obstruction[0] = stoi(rhs.substr(1, place - 1));
				obstruction[1] = stoi(rhs.substr(place + 1, (place2 - place) - 1));
				cout << "obstruction[0] = " << obstruction[0] << endl;
				cout << "obstruction[1] = " << obstruction[1] << endl;
				break;
			case 4:             // AnalysisBoxLeft     ( must be >= 0 and <= 1279)
				AnalysisBoxLeft = stoi(rhs);
				cout << "AnalysisBoxLeft = " << AnalysisBoxLeft << endl;
				break;
			case 5:             // AnalysisBoxTop      ( must be >= 0 and <= 719)
				AnalysisBoxTop = stoi(rhs);
				cout << "AnalysisBoxTop = " << AnalysisBoxTop << endl;
				break;
			case 6:             // AnalysisBoxWidth       ( must be >= 0 and <= (1279 - AnalysisBoxLeft))
				AnalysisBoxWidth = stoi(rhs);
				cout << "AnalysisBoxWidth = " << AnalysisBoxWidth << endl;
				break;
			case 7:             // AnalysisBoxHeight            ( must be >= 0 and <= (719 - AnalysisBoxTop))
				AnalysisBoxHeight = stoi(rhs);
				cout << "AnalysisBoxHeight = " << AnalysisBoxHeight << endl;
				break;
			case 8:             // speedLineLeft              (relative to AnalysisBoxLeft.  Must be >= 0 and <= (719 - AnalysisBoxWidth))
				speedLineLeft = stoi(rhs);
				cout << "speedLineLeft = " << speedLineLeft << endl;
				break;
			case 9:             // speedLineRight          (relative to AnalysisBoxLeft.  Must be > speedLineLeft and <= (719 - AnalysisBoxWidth))
				speedLineRight = stoi(rhs);
				cout << "speedLineRight = " << speedLineRight << endl;
				speedZoneMidPoint = (speedLineRight + speedLineLeft) / 2;  // derived from just read two values for speed zone
				break;
			case 10:             // maxL2RDistOnEntry
				maxL2RDistOnEntry = stoi(rhs);
				cout << "maxL2RDistOnEntry = " << maxL2RDistOnEntry << endl;
				break;
			case 11:             // maxR2LDistOnEntry
				maxR2LDistOnEntry = stoi(rhs);
				cout << "maxR2LDistOnEntry = " << maxR2LDistOnEntry << endl;
				break;
			case 12:             // entryLookBack
				entryLookBack = stoi(rhs);
				cout << "entryLookBack = " << entryLookBack << endl;
				break;
			case 13:             // obstruction_extent
				obstruction_extent = stoi(rhs);
				cout << "obstruction_extent = " << obstruction_extent << endl;
				break;
			case 14:             // largeVehicleArea
				largeVehicleArea = stoi(rhs);
				cout << "largeVehicleArea = " << largeVehicleArea << endl;
				break;
			case 15:             // CalibrationFramesL2R     (How many frames does it take a L2R vehicle to pass thru speed zone at speed limit?)
				CalibrationFramesL2R = stoi(rhs);
				cout << "CalibrationFramesL2R = " << CalibrationFramesL2R << endl;
				break;
			case 16:             // CalibrationFramesR2L     (How many frames does it take a R2L vehicle to pass thru speed zone at speed limit?)
				CalibrationFramesR2L = stoi(rhs);
				cout << "CalibrationFramesR2L = " << CalibrationFramesR2L << endl;
				break;
			case 17:             // SENSITIVITY_VALUE
				SENSITIVITY_VALUE = stoi(rhs);
				cout << "SENSITIVITY_VALUE = " << SENSITIVITY_VALUE << endl;
				break;
			case 18:             // BLUR_SIZE
				BLUR_SIZE = stoi(rhs);
				cout << "BLUR_SIZE = " << BLUR_SIZE << endl;
				break;
			case 19:             // SLOP
				SLOP = stoi(rhs);
				cout << "SLOP = " << SLOP << endl;
				break;
			case 20:             // R2LStreetY         (R2L hubcap line)
				R2LStreetY = stoi(rhs);
				cout << "R2LStreetY = " << R2LStreetY << endl;
				break;
			case 21:             // L2RStreetY         (L2R hubcap line)
				L2RStreetY = stoi(rhs);
				cout << "L2RStreetY = " << L2RStreetY << endl;
				break;
			case 22:             // nextHeight
				nextHeight = stoi(rhs);
				cout << "nextHeight = " << nextHeight << endl;
				break;

			default:
				if (lineNo > 22){
					cout << "Too many lines in config file.  Abortiing." << endl;
					return false;
				}
			} // switch
			lineNo++;
		
		}
		else{
			cout << "getSides() failed in config reader.  Check VST.cfg syntax.  Aborting " << endl;
			configIn.close();
			return false;
		}
		


	}  // while not eof

	configIn.close();

	if ((AnalysisBoxLeft + AnalysisBoxWidth) > 1279){
		cout << "Analysis box too wide.  Must be <= 1279. Check config file.  Aborting." << endl;
		return false;
	}
	else if ((AnalysisBoxTop + AnalysisBoxHeight) > 719){
		cout << "Analysis box too high.  Must be <= 719.  Check config file.  Aborting." << endl;
		return false;
	}
	return true;
};



