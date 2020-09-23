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

int mNumDataPoints = 2788; //max is 2788 apparently

int mNumObstacles = 1;
float mMinObstacleRadius = 2.f;
float mMaxObstacleRadius = 50.f;

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


int main()
{
	mDataTxtFile.open("rrt_data.txt", ios::app);
	mDataCsvFile.open("rrt_data.csv", ios::app);
	////set up headings
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
	

	mDataCsvFile.close();
	mDataTxtFile.close();
	return 0;
}