#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "build_rrt.h"

using namespace std;

ofstream mDataCsvFile;
vector<string> mHeadings;

float mConfSpaceWidth = 600;
float mConfSpaceHeight = 600;

int mNumDataPoints = 0; //max is 2788 apparently

int mNumObstacles = 4;
float mMinObstacleRadius = 10.f;
float mMaxObstacleRadius = 75.f;

int mPercentToDraw = 2;

ofstream mDataResultFile;

void makeNewRRT();
pair<int, float> testRRT(Framework* fw, Vec2 randStart, Vec2 randGoal, Vec2 obs1Pos, float obs1Rad, Vec2 obs2Pos, float obs2Rad, Vec2 obs3Pos, float obs3Rad, Vec2 obs4Pos, float obs4Rad, bool draw);
pair<int, float> sampleRRTOnDistribution(Framework* fw, Vec2 randStart, Vec2 randGoal, Vec2 obs1Pos, float obs1Rad, Vec2 obs2Pos, float obs2Rad, Vec2 obs3Pos, float obs3Rad, Vec2 obs4Pos, float obs4Rad, Vec2 means, float xVar, float yVar, float xyVar, bool draw);
void testTheThing();
void generateData(string dataFile);
void testDataDistAndNonDist(string outputFileName);