#include "rrt_data_generator.h"

void drawRRT(RRT* theRRT) {

}

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

	if (draw) {
		Framework* framework = new Framework(600, 600);;
		myRRT->draw(framework, Vec3(0.f, 1.f, 0.f), Vec3(0.f, 1.f, 1.f), true);

		//show
		std::stringstream sstm;
		sstm << "output/rrtStar.png";
		framework->present_render();
		const char* thing = sstm.str().c_str();
		framework->save_img(thing);

		SDL_Event event = SDL_Event();    // Event variable

		// Below while loop checks if the window has terminated using close in the
		//  corner.
		//event.type = SDL_QUIT;
		while (!(event.type == SDL_QUIT)) {
			SDL_Delay(10);  // setting some Delay
			SDL_PollEvent(&event);  // Catching the poll event.
		}
		delete(framework);
	}

	delete(myRRT);
}

pair<int, float> testRRT(Framework* fw, Vec2 randStart, Vec2 randGoal, Vec2 obs1Pos, float obs1Rad, Vec2 obs2Pos, float obs2Rad, Vec2 obs3Pos, float obs3Rad, Vec2 obs4Pos, float obs4Rad, bool draw) {

		RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);
		myRRT->addObstacle(obs1Pos, obs1Rad);
		myRRT->addObstacle(obs2Pos, obs2Rad);
		myRRT->addObstacle(obs3Pos, obs3Rad);
		myRRT->addObstacle(obs4Pos, obs4Rad);
		
		float calcPercentCovered = ((M_PI * obs1Rad * obs1Rad) + (M_PI * obs2Rad * obs2Rad) + (M_PI * obs3Rad * obs3Rad) + (M_PI * obs4Rad * obs4Rad)) / (600 * 600);
		mDataResultFile << calcPercentCovered << ",";

		//auto solution = myRRT->start();
		auto solution = myRRT->start(true);
		int numNodes = myRRT->getNumNodes();
		mDataResultFile << to_string(numNodes) << ",";

		float totalDist = 0;
		if (solution.size() > 1) {
			for (int i = 0; i < solution.size() - 1; i++) {
				auto point1 = solution[i];
				auto point2 = solution[i + 1];

				totalDist += toVec(point1 - point2).length();
			}
		}
		mDataResultFile << to_string(totalDist) << ",";

		if (draw) {
			myRRT->draw(fw, Vec3(0.f, 1.f, 0.f), Vec3(0.f, 1.f, 1.f), true);
		}

		//TODO:write solution size to file
		if (!myRRT->getIsSuccessful()) numNodes = -1;
		delete myRRT;
		return make_pair(numNodes, totalDist);
}

pair<int, float> sampleRRTOnDistribution(Framework* fw, Vec2 randStart, Vec2 randGoal, Vec2 obs1Pos, float obs1Rad, Vec2 obs2Pos, float obs2Rad, Vec2 obs3Pos, float obs3Rad, Vec2 obs4Pos, float obs4Rad, Vec2 means, float xVar, float yVar, float xyVar, bool draw) {
	RRT* myRRT = new RRT(mConfSpaceWidth, mConfSpaceHeight, randStart, randGoal, 150);
	myRRT->addObstacle(obs1Pos, obs1Rad);
	myRRT->addObstacle(obs2Pos, obs2Rad);
	myRRT->addObstacle(obs3Pos, obs3Rad);
	myRRT->addObstacle(obs4Pos, obs4Rad);

	//auto solution = myRRT->start(means, xVar, yVar, xyVar);
	auto solution = myRRT->start(means, xVar, yVar, xyVar, true);
	int numNodes = myRRT->getNumNodes();
	mDataResultFile << to_string(numNodes) << ",";

	float totalDist = 0;
	if (solution.size() > 1) {
		for (int i = 0; i < solution.size() - 1; i++) {
			auto point1 = solution[i];
			auto point2 = solution[i + 1];

			totalDist += toVec(point1 - point2).length();
		}
	}
	mDataResultFile << to_string(totalDist) << ",";
	bool successful = myRRT->getIsSuccessful();
	mDataResultFile << successful << ",";
	if (!successful) {
		myRRT->draw(fw, Vec3(1.0f, 0.f, 1.f), Vec3(0.5f, 0.f, 1.f), false, true, Vec3(1.f, 0.f, 0.f), means, xVar, yVar, xyVar);
	}
	else if (draw) {
		myRRT->draw(fw, Vec3(1.0f, 0.f, 1.f), Vec3(0.5f, 0.f, 1.f), false, true, Vec3(1.f, 0.f, 0.f), means, xVar, yVar, xyVar);
	}
	//TODO:write solution size to file
	if (!myRRT->getIsSuccessful()) numNodes = -1;
	return make_pair(numNodes, totalDist);
	delete myRRT;
}

void testTheThing() {
	ifstream testData;
	testData.open("rrt_data_predicted_4obs.csv");
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
	//mDataResultFile << endl;
	int count = 0;
	std::string buffer;
	size_t strpos = 0;
	size_t endpos;
	while (getline(testData, buffer))
	{
	// Read data, line by line
		//while (getline(testData, line))
		//{
			cout << "on dist: working on line " << count << endl;
			// Create a stringstream of the current line
			stringstream ss(line);

			// Keep track of the current column index
			int colIdx = 0;

			Vec2 randStart = Vec2(-1.f, -1.f);
			Vec2 randGoal = Vec2(-1.f, -1.f);

			Vec2 obs1Pos = Vec2(-1.f, -1.f);
			float obs1Rad = -1;
			Vec2 obs2Pos = Vec2(-1.f, -1.f);
			float obs2Rad = -1;
			Vec2 obs3Pos = Vec2(-1.f, -1.f);
			float obs3Rad = -1;
			Vec2 obs4Pos = Vec2(-1.f, -1.f);
			float obs4Rad = -1;
			Vec2 means = Vec2(-1.f, -1.f);
			float xVar = -1;
			float yVar = -1;
			float xyVar = -1;

			colIdx = 0;
			endpos = buffer.find(',');
			while (endpos < buffer.length())
			{
				string strVal = buffer.substr(strpos, endpos - strpos);
				float val = std::stof(strVal);

				switch (colIdx) {
				case 0:
					//start x
					randStart.setVal(0, 0, val);
					break;
				case 1:
					//start y
					randStart.setVal(1, 0, val);
					break;
				case 2:
					//goal x
					randGoal.setVal(0, 0, val);
					break;
				case 3:
					//goal y
					randGoal.setVal(1, 0, val);
					break;
				case 4:
					//obs 1 x
					obs1Pos.setVal(0, 0, val);
					break;
				case 5:
					//obs 1 y
					obs1Pos.setVal(1, 0, val);
					break;
				case 6:
					//obs 1 rad
					obs1Rad = val;
					break;
				case 7:
					//obs 2 x
					obs2Pos.setVal(0, 0, val);
					break;
				case 8:
					//obs 2 y
					obs2Pos.setVal(1, 0, val);
					break;
				case 9:
					//obs 2 rad
					obs2Rad = val;
					break;
				case 10:
					//obs 3 x
					obs3Pos.setVal(0, 0, val);
					break;
				case 11:
					//obs 3 y
					obs3Pos.setVal(1, 0, val);
					break;
				case 12:
					//obs 3 rad
					obs3Rad = val;
					break;
				case 13:
					//obs 4 x
					obs4Pos.setVal(0, 0, val);
					break;
				case 14:
					//obs 4 y
					obs4Pos.setVal(1, 0, val);
					break;
				case 15:
					//obs 4 rad
					obs4Rad = val;
					break;
				case 16:
					//generated x means
					means.setVal(0, 0, val);
					break;
				case 17:
					//generated y means
					means.setVal(1, 0, val);
					break;
				case 18:
					//generated x variance
					xVar = (float)val;
					break;
				case 19:
					//generated y variance
					yVar = (float)val;
					break;
				case 20:
					//generated xy variance
					xyVar = (float)val;
					break;
				default:
					break;
				}

				colIdx++;
				strpos = endpos + 1;
				endpos = buffer.find(',', strpos);
			}
			xyVar = std::stof(buffer.substr(strpos, endpos - strpos));
			strpos = endpos + 1;
			endpos = buffer.find(',', strpos);

			bool todraw = (rand() % 100 < mPercentToDraw);

			Framework* framework = new Framework(600, 600);;

			//returns num nodes sampled, path length
			pair<int, float> withoutDistNum = testRRT(framework, randStart, randGoal, obs1Pos, obs1Rad, obs2Pos, obs2Rad, obs3Pos, obs3Rad, obs4Pos, obs4Rad, todraw);
			pair<int, float> withDistNum = sampleRRTOnDistribution(framework, randStart, randGoal, obs1Pos, obs1Rad, obs2Pos, obs2Rad, obs3Pos, obs3Rad, obs4Pos, obs4Rad, means, xVar, yVar, xyVar, todraw);
			bool successful = (withDistNum.first >= 0);

			int nodediff = withDistNum.first - withoutDistNum.first;
			float nodepercentDecrease = (float)nodediff / (float)withDistNum.first;

			float pathdiff = withDistNum.second - withoutDistNum.second;
			float pathpercentDecrease = (float)pathdiff / (float)withDistNum.second;

			mDataResultFile << to_string(pathdiff) << ", " << to_string(pathpercentDecrease) << ",";
			mDataResultFile << to_string(nodediff) << ", " << to_string(nodepercentDecrease);
			mDataResultFile << endl;

			//show
			if (todraw) {
				std::stringstream sstm;
				sstm << "output/" << count << "_dist.png";
				framework->present_render();
				const char* thing = sstm.str().c_str();
				framework->save_img(thing);
			}

			SDL_Event event = SDL_Event();    // Event variable

			// Below while loop checks if the window has terminated using close in the
			//  corner.
			event.type = SDL_QUIT;
			while (!(event.type == SDL_QUIT)) {
				SDL_Delay(10);  // setting some Delay
				SDL_PollEvent(&event);  // Catching the poll event.
			}
			delete(framework);
			count++;
		}

		//cout << ss.str() << endl;
		// Extract each integer
		//while (ss >> val) {

		

		//	// If the next token is a comma, ignore it and move on
		//	if (ss.peek() == ',') ss.ignore();

		//	// Increment the column index
		//	colIdx++;
		//}


		
	//}
	testData.close();
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

	mHeadings.push_back("\"Predicted Mean X\"");
	mHeadings.push_back("\"Predicted Mean Y\"");
	mHeadings.push_back("\"Predicted SXX\"");
	mHeadings.push_back("\"Predicted SYY\"");
	mHeadings.push_back("\"Predicted SXY\"");

	for (int j = 0; j < mHeadings.size(); ++j) {
		mDataCsvFile << mHeadings.at(j);
		if (j != mHeadings.size() - 1) {
			mDataCsvFile << ","; // No comma at end of line
		}
	}
	mDataCsvFile << endl;

	for (int i = 0; i < mNumDataPoints; i++) {
		cout << "making situation " << i << " : ";
		makeNewRRT(false);
		mDataCsvFile << endl;

	}

	mDataCsvFile.close();
}

void testDataDistAndNonDist(string outputFileName) {
	mDataResultFile.open(outputFileName);
	mDataResultFile << "area covered, without dist, without dist length, with dist, with dist length, successful, path length diff, path length percent decrease, node num diff, node num percent decrease" << endl;

	testTheThing();

	mDataResultFile.close();
}

int main(int argc, char* argv[]) {
	generateData("rrtStar_data.csv");
	//testDataDistAndNonDist("rrt_test_result_data.csv");
	
	return 0;
}