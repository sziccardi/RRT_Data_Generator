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

int mNumDataPoints = 1000; //max is 2788 apparently

int mNumObstacles = 4;
float mMinObstacleRadius = 10.f;
float mMaxObstacleRadius = 75.f;

int mPercentToDraw = 2;

ofstream mDataResultFile;

void makeNewRRT(bool draw = false);
void generateData(string dataFile);