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

string line, lhs, rhs;


bool getSides(string inLine){
	string tempLHS, tempRHS;
//	cout << "Inside getSides()" << endl;
	istringstream configLine(inLine);
	getline(configLine, tempLHS, '=');
	getline(configLine, tempRHS, '#');
	lhs = tempLHS;
	rhs = tempRHS;
//	cout << "Returning from getSides()<" << lhs << " >  <" << rhs << endl;
	return true;
}

string trim(string toBeTrimmed){
//	cout << "Inside trim()" << endl;
	int first = 0;
	while (toBeTrimmed.substr(first, 1) == " ") first++;
	int last = toBeTrimmed.length() - 1;
	while (toBeTrimmed.substr(last, 1) == " ") last--;
//	cout << "Returning from trim() " << first << "  " << last << endl;
	return toBeTrimmed.substr(first, (last-first)+1);
}



bool Globals::readConfig(){
	ifstream configIn("VST.cfg");
	if (!configIn.good()){
		cout << "Can't open VST.cfg." << endl;
		return false;
	}
	//int lineCount = 1;
	//getline(configIn, line);
	//if (!configIn){
	//	return false;
	//}
	//istringstream configLine(line);
	//getline(configLine, lhs, '=');
	//getline(configLine, rhs, '#');
	//cout << "First line of config file: " << lhs << "  " << rhs << endl;
	//if (trim(lhs) == "AnalysisBoxLeft"){
	//	AnalysisBoxLeft = stoi(trim(rhs));
	//	cout << "AnalysisBoxLeft just set to: " << AnalysisBoxLeft << endl;
	//}
	//else{
	//	configIn.close();
	//	return false;
	//}

	//lineCount++;
	//getline(configIn, line);
	//if (!configIn){
	//	configError(lineCount);
	//	return false;
	//}
	//istringstream configLine2(line);
	//getline(configLine2, lhs, '=');
	//getline(configLine2, rhs, '#');
	//cout << "Second line of config file: " << lhs << "  " << rhs << endl;
	//if (trim(lhs) == "AnalysisBoxTop"){
	//	AnalysisBoxTop = stoi(trim(rhs));
	//	cout << "AnalysisBoxTop just set to: " << AnalysisBoxTop << endl;
	//}
	//else {
	//	configIn.close();
	//	return false;
	//}





	string lhsString[23] = {
		"dataPathPrefix",
		"L2RDirection",
		"R2LDirection",
		"obstruction",     /// ************************************************

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
	// line == line
	// is =  configIn
	// iss = configLine

	int place, place2;
	int lineNo = 0;
	while (!configIn.eof()){
		getline(configIn, line);
		if (!configIn){
			cout << "Unexpected error in config file." << endl;
			return false;
		}
//		cout << "Inside while loop. just read line is:" << line << endl;
		if (getSides(line)){

//			cout << "LHS/RHS: <" << lhs << "> <" << rhs << ">" << endl;
			lhs = trim(lhs);
			rhs = trim(rhs);
			switch (lineNo){
			case 0:             // dataPathPrefix
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
			case 3:             // obstruction
				place = rhs.find(',');
				place2 = rhs.find(']');
				obstruction[0] = stoi(rhs.substr(1, place - 1));
				obstruction[1] = stoi(rhs.substr(place + 1, (place2 - place) - 1));
				cout << "obstruction[0] = " << obstruction[0] << endl;
				cout << "obstruction[1] = " << obstruction[1] << endl;
				break;
			case 4:             // AnalysisBoxLeft
				AnalysisBoxLeft = stoi(rhs);
				cout << "AnalysisBoxLeft = " << AnalysisBoxLeft << endl;
				break;
			case 5:             // AnalysisBoxTop
				AnalysisBoxTop = stoi(rhs);
				cout << "AnalysisBoxTop = " << AnalysisBoxTop << endl;
				break;
			case 6:             // AnalysisBoxWidth
				AnalysisBoxWidth = stoi(rhs);
				cout << "AnalysisBoxWidth = " << AnalysisBoxWidth << endl;
				break;
			case 7:             // AnalysisBoxHeight
				AnalysisBoxHeight = stoi(rhs);
				cout << "AnalysisBoxHeight = " << AnalysisBoxHeight << endl;
				break;
			case 8:             // speedLineLeft
				speedLineLeft = stoi(rhs);
				cout << "speedLineLeft = " << speedLineLeft << endl;
				break;
			case 9:             // speedLineRight
				speedLineRight = stoi(rhs);
				cout << "speedLineRight = " << speedLineRight << endl;
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
			case 15:             // CalibrationFramesL2R
				CalibrationFramesL2R = stoi(rhs);
				cout << "CalibrationFramesL2R = " << CalibrationFramesL2R << endl;
				break;
			case 16:             // CalibrationFramesR2L
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
			case 20:             // R2LStreetY
				R2LStreetY = stoi(rhs);
				cout << "R2LStreetY = " << R2LStreetY << endl;
				break;
			case 21:             // L2RStreetY
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
				else if (lhs != lhsString[lineNo]){
					cout << "Just read LHS doesn't match anything: <" << lhs << ">" << endl;
					return false;
				}

			} // switch
			lineNo++;
		
		}
		else{
			cout << "getSides() failed in config reader. " << endl;
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



