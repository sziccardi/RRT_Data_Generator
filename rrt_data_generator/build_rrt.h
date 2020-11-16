#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <random>
#include <iomanip>

#include <objidl.h>

#include "data_utils.h"

using namespace std;

class RRT {
public:
	RRT();
	RRT(int confWidth, int confHeight, Vec2 startPos, Vec2 goalPos, int numVerts);

	Vec2 getInitPos();
	void setInitPos(Vec2 initPos);

	Vec2 getGoalPos();
	void setGoalPos(Vec2 goalPos);

	float getConfWidth();
	float getConfHeight();
	void setConfSize(float width, float height);

	int getNumVertices();
	void setNumVertices(int numVerts);

	int getNumNodes();
	int getIsSuccessful();

	void addObstacle(Vec2 pos, float radius);
	vector<Vec2> start();

	vector<Vec2> start(Vec2 means, float sxx, float syy, float sxy);

	void draw(Framework* fw, Vec3 solutionColor, Vec3 treeColor, bool drawObs, bool drawDist = false, Vec3 distColor = Vec3(-1.f, -1.f, -1.f), Vec2 means = Vec2(-1.f, -1.f), float sxx = -1.f, float syy = -1.f, float sxy = -1.f);

	bool mUseDist = false;
	int mCountMax = 10000;

private:
	Vec2 randConfEven();
	Vec2 randConfDist(Vec2 means = Vec2(-1, -1), float sxx = -1, float syy = -1, float sxy = -1);

	Node* nearestNode(Vec2 rand);
	Vec2 newConf(Vec2 nearby, Vec2 rand);

	void letsBuildRRTOnDist(Vec2 means, float sxx, float syy, float sxy);
	void letsBuildRRT();
	void initEnvironment();

	int mNumVertices = 154;
	Tree* myTree;
	vector<Vec2> mSolutionPath;
	Vec2 mInitPos = Vec2(-1.0, -1.0);
	Vec2 mGoalPos = Vec2(-1.0, -1.0);
	vector<pair<Vec2, float>> mObstacles;
	float dq = 10;
	int mWidth = 0;
	int mHeight = 0;
};