#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "build_rrt.cpp"

#include <SDL.h>

using namespace std;

ofstream mDataTxtFile;
ofstream mDataCsvFile;
vector<string> mHeadings;

float mConfSpaceWidth = 600;
float mConfSpaceHeight = 600;

int mNumDataPoints = 10000; //max is 2788 apparently

int mNumObstacles = 0;
float mMinObstacleRadius = 10.f;
float mMaxObstacleRadius = 75.f;

vector<float> withDistVals;
vector<float> withoutDistVals;

void makeNewRRT() {
	vec2 randStart = vec2(0.f, rand() % (int)mConfSpaceHeight);
	vec2 randGoal = vec2(mConfSpaceWidth, rand() % (int)mConfSpaceHeight);
	mDataTxtFile << randStart.mX << "," << randStart.mY << "," << randGoal.mX << "," << randGoal.mY;
	mDataCsvFile << randStart.mX << "," << randStart.mY << "," << randGoal.mX << "," << randGoal.mY;
	if (mNumObstacles > 0) {
		mDataTxtFile << ",";
		mDataCsvFile << ",";
	}
	RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);

	for (int i = 0; i < mNumObstacles; i++) {
		float radius = rand() % (int)(mMaxObstacleRadius - mMinObstacleRadius) + mMinObstacleRadius;
		vec2 randObstacle = vec2(rand() % (int)(mConfSpaceWidth - 2 * radius) + radius, rand() % (int)(mConfSpaceHeight - 2 * radius) + radius);
		mDataTxtFile << randObstacle.mX << "," << randObstacle.mY << "," << radius << ",";
		mDataCsvFile << randObstacle.mX << "," << randObstacle.mY << "," << radius << ",";
		myRRT->addObstacle(randObstacle, radius);
	}
	auto solution = myRRT->start();

	float numSamples = (float)solution.size();
	float xMean = 0.f;
	float yMean = 0.f;
	for (auto point : solution) {
		xMean += point.mX;
		yMean += point.mY;
	}
	xMean /= numSamples;
	yMean /= numSamples;
	mDataTxtFile << xMean << ",";
	mDataCsvFile << xMean << ",";
	mDataTxtFile << yMean << ",";
	mDataCsvFile << yMean << ",";


	float sXY = 0.f;
	float sXX = 0.f;
	float sYY = 0.f;
	for (auto point : solution) {
		float xVar = point.mX - xMean;
		float yVar = point.mY - yMean;

		sXY += (xVar * yVar);
		sXX += (xVar * xVar);
		sYY += (yVar * yVar);
	}

	sXY /= (numSamples - 1);
	sXX /= (numSamples - 1);
	sYY /= (numSamples - 1);

	mDataTxtFile << sXX << ",";
	mDataCsvFile << sXX << ",";
	mDataTxtFile << sYY << ",";
	mDataCsvFile << sYY << ",";
	mDataTxtFile << sXY;
	mDataCsvFile << sXY;
}

void testRRT() {
	ifstream testData;
	testData.open("rrt_data.csv");
	if (!testData.is_open()) {
		std::cout << "Could not open file" << endl;
		return;
	}
	// Helper vars
	std::string line, colname;
	int val;

	// Read the column names
	if (testData.good()) {
		// Extract the first line in the file
		getline(testData, line);
	}

	// Read data, line by line
	while (getline(testData, line))
	{
		// Create a stringstream of the current line
		stringstream ss(line);

		// Keep track of the current column index
		int colIdx = 0;

		vec2 randStart = vec2(-1.f, -1.f);
		vec2 randGoal = vec2(-1.f, -1.f);

		vec2 obs1Pos = vec2(-1.f, -1.f);
		float obs1Rad = -1;
		vec2 obs2Pos = vec2(-1.f, -1.f);
		float obs2Rad = -1;
		vec2 obs3Pos = vec2(-1.f, -1.f);
		float obs3Rad = -1;
		vec2 obs4Pos = vec2(-1.f, -1.f);
		float obs4Rad = -1;

		// Extract each integer
		while (ss >> val) {

			switch (colIdx) {
			case 0:
				//start x
				randStart.mX = val;
				break;
			case 1:
				//start y
				randStart.mY = val;
				break;
			case 2:
				//goal x
				randGoal.mX = val;
				break;
			case 3:
				//goal y
				randGoal.mY = val;
				break;
			case 4:
				//obs 1 x
				obs1Pos.mX = val;
				break;
			case 5:
				//obs 1 y
				obs1Pos.mY = val;
				break;
			case 6:
				//obs 1 rad
				obs1Rad = val;
				break;
			case 7:
				//obs 2 x
				obs2Pos.mX = val;
				break;
			case 8:
				//obs 2 y
				obs2Pos.mY = val;
				break;
			case 9:
				//obs 2 rad
				obs2Rad = val;
				break;
			case 10:
				//obs 3 x
				obs3Pos.mX = val;
				break;
			case 11:
				//obs 3 y
				obs3Pos.mY = val;
				break;
			case 12:
				//obs 3 rad
				obs3Rad = val;
				break;
			case 13:
				//obs 4 x
				obs4Pos.mX = val;
				break;
			case 14:
				//obs 4 y
				obs4Pos.mY = val;
				break;
			case 15:
				//obs 4 rad
				obs4Rad = val;
				break;
			default:
				break;
			}

			// If the next token is a comma, ignore it and move on
			if (ss.peek() == ',') ss.ignore();

			// Increment the column index
			colIdx++;
		}


		RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);
		myRRT->addObstacle(obs1Pos, obs1Rad);
		myRRT->addObstacle(obs2Pos, obs2Rad);
		myRRT->addObstacle(obs3Pos, obs3Rad);
		myRRT->addObstacle(obs4Pos, obs4Rad);

		auto solution = myRRT->start();
		withoutDistVals.push_back(myRRT->getNumNodes());

		myRRT->draw();

		//TODO:write solution size to file
	}
	testData.close();
}

void sampleRRTOnDistribution() {
	ifstream testData;
	testData.open("rrt_data.csv");
	if (!testData.is_open()) {
		std::cout << "Could not open file" << endl;
		return;
	}
	// Helper vars
	std::string line, colname;
	int val;

	// Read the column names
	if (testData.good()) {
		// Extract the first line in the file
		getline(testData, line);
	}

	// Read data, line by line
	while (getline(testData, line))
	{
		// Create a stringstream of the current line
		stringstream ss(line);

		// Keep track of the current column index
		int colIdx = 0;

		vec2 randStart = vec2(-1.f, -1.f);
		vec2 randGoal = vec2(-1.f, -1.f);

		vec2 obs1Pos = vec2(-1.f, -1.f);
		float obs1Rad = -1;
		vec2 obs2Pos = vec2(-1.f, -1.f);
		float obs2Rad = -1;
		vec2 obs3Pos = vec2(-1.f, -1.f);
		float obs3Rad = -1;
		vec2 obs4Pos = vec2(-1.f, -1.f);
		float obs4Rad = -1;
		vec2 means = vec2(-1.f, -1.f);
		float xVar = -1;
		float yVar = -1;
		float xyVar = -1;

		// Extract each integer
		while (ss >> val) {

			switch (colIdx) {
			case 0:
				//start x
				randStart.mX = val;
				break;
			case 1:
				//start y
				randStart.mY = val;
				break;
			case 2:
				//goal x
				randGoal.mX = val;
				break;
			case 3:
				//goal y
				randGoal.mY = val;
				break;
			case 4:
				//obs 1 x
				obs1Pos.mX = val;
				break;
			case 5:
				//obs 1 y
				obs1Pos.mY = val;
				break;
			case 6:
				//obs 1 rad
				obs1Rad = val;
				break;
			case 7:
				//obs 2 x
				obs2Pos.mX = val;
				break;
			case 8:
				//obs 2 y
				obs2Pos.mY = val;
				break;
			case 9:
				//obs 2 rad
				obs2Rad = val;
				break;
			case 10:
				//obs 3 x
				obs3Pos.mX = val;
				break;
			case 11:
				//obs 3 y
				obs3Pos.mY = val;
				break;
			case 12:
				//obs 3 rad
				obs3Rad = val;
				break;
			case 13:
				//obs 4 x
				obs4Pos.mX = val;
				break;
			case 14:
				//obs 4 y
				obs4Pos.mY = val;
				break;
			case 15:
				//obs 4 rad
				obs4Rad = val;
				break;
			case 16:
				//generated x means
				means.mX = val;
				break;
			case 17:
				//generated y means
				means.mY = val;
				break;
			case 18:
				//generated x variance
				xVar = val;
				break;
			case 19:
				//generated y variance
				yVar = val;
				break;
			case 20:
				//generated xy variance
				xyVar = val;
				break;
			default:
				break;
			}

			// If the next token is a comma, ignore it and move on
			if (ss.peek() == ',') ss.ignore();

			// Increment the column index
			colIdx++;
		}


		RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);
		myRRT->addObstacle(obs1Pos, obs1Rad);
		myRRT->addObstacle(obs2Pos, obs2Rad);
		myRRT->addObstacle(obs3Pos, obs3Rad);
		myRRT->addObstacle(obs4Pos, obs4Rad);

		auto solution = myRRT->start(means, xVar, yVar, xyVar);
		withDistVals.push_back(myRRT->getNumNodes());
		myRRT->draw();

		//TODO:write solution size to file
	}
	testData.close();
}


int main(int argc, char* argv[])
{
	mDataTxtFile.open("rrt_data.txt");
	mDataCsvFile.open("rrt_data.csv");
	//set up headings
	mHeadings.clear();
	mHeadings.push_back("\"Start Loc X\"");
	mHeadings.push_back("\"Start Loc Y\"");
	mHeadings.push_back("\"Goal Loc X\"");
	mHeadings.push_back("\"Goal Loc Y\"");
	for (int i = 0; i < mNumObstacles; i++) {
		mHeadings.push_back("\"Obstacle " + to_string(i + 1) + " Loc X\"");
		mHeadings.push_back("\"Obstacle " + to_string(i + 1) + " Loc Y\"");
		mHeadings.push_back("\"Obstacle " + to_string(i + 1) + " Radius\"");
	}

	mHeadings.push_back("\"Mean X\"");
	mHeadings.push_back("\"Mean Y\"");
	mHeadings.push_back("\"SXX\"");
	mHeadings.push_back("\"SYY\"");
	mHeadings.push_back("\"SXY\"");

	for (int j = 0; j < mHeadings.size(); ++j) {
		mDataTxtFile << mHeadings.at(j);
		mDataCsvFile << mHeadings.at(j);
		if (j != mHeadings.size() - 1) {
			mDataCsvFile << ","; // No comma at end of line
			mDataTxtFile << ","; // No comma at end of line
		}
	}
	mDataCsvFile << endl;
	mDataTxtFile << endl;

	for (int i = 0; i < mNumDataPoints; i++) {
		cout << "making situation " << i << " : ";
		makeNewRRT();
		mDataCsvFile << endl;
		mDataTxtFile << endl;
	}
	/*testRRT();
	sampleRRTOnDistribution();

	ofstream testResultData;
	testResultData.open("rrt_test_result_data.csv");
	testResultData << "without Dist, with Dist, diff, percent decrease" << endl;
	for (int i = 0; i < withDistVals.size(); i++) {
		testResultData << to_string(withoutDistVals[i]) << ",";
		testResultData << to_string(withDistVals[i]) << ",";
		testResultData << to_string(withoutDistVals[i] - withDistVals[i]) << ",";
		testResultData << to_string((withoutDistVals[i] - withDistVals[i]) / withoutDistVals[i]) << endl;
	}
	testResultData.close();
	mDataCsvFile.close();
	mDataTxtFile.close();*/

	return 0;
}