#include "rrt_data_generator.h"


void makeNewRRT(bool draw) {
	Vec2 randStart = Vec2(0.f, rand() % (int)mConfSpaceHeight);
	Vec2 randGoal = Vec2(mConfSpaceWidth, rand() % (int)mConfSpaceHeight);
	mDataCsvFile << randStart.x() << "," << randStart.y() << "," << randGoal.x() << "," << randGoal.y();
	if (mNumObstacles > 0) {
		mDataCsvFile << ",";
	}
	RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);

	int actualNumObs = rand() % (mNumObstacles + 1);
	int actualNumNoObs = mNumObstacles - actualNumObs;
	for (int i = 0; i < actualNumObs; i++) {
		float radius = rand() % (int)(mMaxObstacleRadius - mMinObstacleRadius) + mMinObstacleRadius;
		Vec2 randObstacle = Vec2(rand() % (int)(mConfSpaceWidth - 2 * radius) + radius, rand() % (int)(mConfSpaceHeight - 2 * radius) + radius);
		mDataCsvFile << randObstacle.x() << "," << randObstacle.y() << "," << radius << ",";
		myRRT->addObstacle(randObstacle, radius);
	}
	for (int i = 0; i < actualNumNoObs; i++) {
		float radius = 0.f;
		Vec2 randObstacle = Vec2(rand() % (int)(mConfSpaceWidth - 2 * radius) + radius, rand() % (int)(mConfSpaceHeight - 2 * radius) + radius);
		mDataCsvFile << randObstacle.x() << "," << randObstacle.y() << "," << radius << ",";
		myRRT->addObstacle(randObstacle, radius);
	}
	//auto solution = myRRT->start();
	auto solution = myRRT->start(true);

	float numSamples = (float)solution.size();
	float xMean = -1.f;
	float yMean = -1.f;
	if (numSamples > 0) {
		xMean = 0.f;
		yMean = 0.f;
		for (auto point : solution) {
			xMean += point.x();
			yMean += point.y();
		}
		xMean /= numSamples;
		yMean /= numSamples;
	}
	mDataCsvFile << xMean << ",";
	mDataCsvFile << yMean << ",";


	float sXY = -1.f;
	float sXX = -1.f;
	float sYY = -1.f;
	if (numSamples > 0) {
		sXY = 0.f;
		sXX = 0.f;
		sYY = 0.f;
		for (auto point : solution) {
			float xVar = point.x() - xMean;
			float yVar = point.y() - yMean;

			sXY += (xVar * yVar);
			sXX += (xVar * xVar);
			sYY += (yVar * yVar);
		}

		sXY /= (numSamples - 1);
		sXX /= (numSamples - 1);
		sYY /= (numSamples - 1);
	}
	mDataCsvFile << sXX << ",";
	mDataCsvFile << sYY << ",";
	mDataCsvFile << sXY;

	delete(myRRT);
}

void generateData(string dataFile) {
	mDataCsvFile.open(dataFile);
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

	mHeadings.push_back("\"Predicted Mean 1 X\"");
	mHeadings.push_back("\"Predicted Mean 1 Y\"");
	mHeadings.push_back("\"Predicted SXX 1\"");
	mHeadings.push_back("\"Predicted SYY 1\"");
	mHeadings.push_back("\"Predicted SXY 1\"");

	mHeadings.push_back("\"Predicted Mean 1 X\"");
	mHeadings.push_back("\"Predicted Mean 1 Y\"");
	mHeadings.push_back("\"Predicted SXX 1\"");
	mHeadings.push_back("\"Predicted SYY 1\"");
	mHeadings.push_back("\"Predicted SXY 1\"");

	for (int j = 0; j < mHeadings.size(); ++j) {
		mDataCsvFile << mHeadings.at(j);
		if (j != mHeadings.size() - 1) {
			mDataCsvFile << ","; // No comma at end of line
		}
	}
	mDataCsvFile << endl;

	for (int i = 0; i < mNumDataPoints; i++) {
		cout << "making situation " << i << " : ";
		int choice = rand() % 100;
		bool draw = (choice < 20);
		makeNewRRT(false);
		mDataCsvFile << endl;

	}

	mDataCsvFile.close();
}

int main(int argc, char* argv[]) {
	generateData("rrtStar_data.csv");
	//testDataDistAndNonDist("rrt_test_result_data.csv");
	
	return 0;
}