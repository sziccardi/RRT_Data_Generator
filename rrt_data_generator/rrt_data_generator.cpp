#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "build_rrt.cpp"

using namespace std;

ofstream mDataTxtFile;
ofstream mDataCsvFile;
vector<string> mHeadings;

float mConfSpaceWidth = 600;
float mConfSpaceHeight = 600;

int mNumDataPoints = 10;

int mNumObstacles = 1;
float mMinObstacleRadius = 2.f;
float mMaxObstacleRadius = 50.f;

int mMaxNumSolutionPoints = 300;

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
	mDataTxtFile << solution.size() << ",";
	mDataCsvFile << solution.size() << ",";
	for (int i = mMaxNumSolutionPoints - 1; i >= 0; i--) {
		if (i < solution.size()) {
			mDataTxtFile << solution.at(i).mX << "," << solution.at(i).mY;
			mDataCsvFile << solution.at(i).mX << "," << solution.at(i).mY;
		} else {
			mDataTxtFile << randGoal.mX << "," << randGoal.mY;
			mDataCsvFile << randGoal.mX << "," << randGoal.mY;
		}
		if (i > 0) {
			mDataTxtFile << ",";
			mDataCsvFile << ",";
		}
	}
}


int main()
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
	mHeadings.push_back("\"Length of Solution Path\"");
	///TODO: decide how to represent solution path
	/// string of numbers?
	/// max number of points accepted, remove any data that goes above that max?
	/// image?
	/// lets try max points first

	for (int i = 0; i < mMaxNumSolutionPoints; i++) {
		mHeadings.push_back("\"Solution Loc " + to_string(i + 1) + " X\"");
		mHeadings.push_back("\"Solution Loc " + to_string(i + 1) + " Y\"");
	}

	for (int j = 0; j < mHeadings.size(); ++j) {
		mDataTxtFile << mHeadings.at(j);
		mDataTxtFile << mHeadings.at(j);
		if (j != mHeadings.size() - 1) {
			mDataCsvFile << ","; // No comma at end of line
			mDataTxtFile << ","; // No comma at end of line
		}
	}
	mDataCsvFile << endl;
	mDataTxtFile << endl;

	for (int i = 0; i < mNumDataPoints; i++) {
		makeNewRRT();
		mDataCsvFile << endl;
		mDataTxtFile << endl;
	}
	

	mDataCsvFile.close();
	mDataTxtFile.close();
	return 0;
}