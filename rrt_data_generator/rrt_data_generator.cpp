#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "build_rrt.cpp"

using namespace std;

ofstream mDataFile;
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
	mDataFile << randStart.mX << "," << randStart.mY << "," << randGoal.mX << "," << randGoal.mY;
	if (mNumObstacles > 0) mDataFile << ",";
	RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);

	for (int i = 0; i < mNumObstacles; i++) {
		float radius = rand() % (int)(mMaxObstacleRadius - mMinObstacleRadius) + mMinObstacleRadius;
		vec2 randObstacle = vec2(rand() % (int)(mConfSpaceWidth - 2 * radius) + radius, rand() % (int)(mConfSpaceHeight - 2 * radius) + radius);
		mDataFile << randObstacle.mX << "," << randObstacle.mY << "," << radius << ",";
		myRRT->addObstacle(randObstacle, radius);
	}
	auto solution = myRRT->start();
	mDataFile << solution.size() << ",";
	for (int i = mMaxNumSolutionPoints - 1; i >= 0; i--) {
		if (i < solution.size()) {
			mDataFile << solution.at(i).mX << "," << solution.at(i).mY;
		} else {
			mDataFile << randGoal.mX << "," << randGoal.mY;
		}
		if (i > 0) {
			mDataFile << ",";
		}
	}
}


int main()
{
	mDataFile.open("rrt_data.txt");
	//set up headings
	mHeadings.clear();
	mHeadings.push_back("Start Loc X");
	mHeadings.push_back("Start Loc Y");
	mHeadings.push_back("Goal Loc X");
	mHeadings.push_back("Goal Loc Y");
	for (int i = 0; i < mNumObstacles; i++) {
		mHeadings.push_back("Obstacle " + to_string(i + 1) + " Loc X");
		mHeadings.push_back("Obstacle " + to_string(i + 1) + " Loc Y");
		mHeadings.push_back("Obstacle " + to_string(i + 1) + " Radius");
	}
	mHeadings.push_back("Length of Solution Path");
	///TODO: decide how to represent solution path
	/// string of numbers?
	/// max number of points accepted, remove any data that goes above that max?
	/// image?
	/// lets try max points first

	for (int i = 0; i < mMaxNumSolutionPoints; i++) {
		mHeadings.push_back("Solution Loc " + to_string(i + 1) + " X");
		mHeadings.push_back("Solution Loc " + to_string(i + 1) + " Y");
	}

	for (int j = 0; j < mHeadings.size(); ++j) {
		mDataFile << mHeadings.at(j);
		if (j != mHeadings.size() - 1) mDataFile << ","; // No comma at end of line
	}
	mDataFile << endl;

	for (int i = 0; i < mNumDataPoints; i++) {
		makeNewRRT();
		mDataFile << endl;
	}
	

	mDataFile.close();
	return 0;
}