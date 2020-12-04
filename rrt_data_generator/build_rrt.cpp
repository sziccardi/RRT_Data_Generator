#include "build_rrt.h"

RRT::RRT() { initEnvironment(); }

RRT::RRT(int confWidth, int confHeight, Vec2 startPos, Vec2 goalPos, int numVerts) {
	mWidth = confWidth;
	mHeight = confHeight;

	mInitPos = startPos;
	mGoalPos = goalPos;

	mNumVertices = numVerts;
	initEnvironment();
	return;
}

RRT::~RRT() {
	if (myTree) {
		auto t = myTree;
		delete(t);
		myTree = nullptr;
	}
}

Vec2 RRT::getInitPos() { return mInitPos; }
void RRT::setInitPos(Vec2 initPos) { mInitPos = initPos; }

Vec2 RRT::getGoalPos() { return mGoalPos; }
void RRT::setGoalPos(Vec2 goalPos) { mGoalPos = goalPos; }

float RRT::getConfWidth() { return mWidth; }
float RRT::getConfHeight() { return mHeight; }
void RRT::setConfSize(float width, float height) { mWidth = width; mHeight = height; }

int RRT::getNumVertices() { return mNumVertices; }
void RRT::setNumVertices(int numVerts) { mNumVertices = numVerts; }
	
int RRT::getNumNodes() { return myTree->getTreeSize(); }
int RRT::getIsSuccessful() { return (mSolutionPath.size() > 0); }

void RRT::addObstacle(Vec2 pos, float radius) {
	mObstacles.push_back(make_pair(pos, radius));
}

vector<Vec2> RRT::start(bool useStar) {
	if (useStar) {
		letsBuildRRTStar();
	} else {
		letsBuildRRT();
	}
	return mSolutionPath;
}

vector<Vec2> RRT::start(Vec2 means, float sxx, float syy, float sxy, bool useStar) {
	if (useStar) {
		letsBuildRRTStar();
	} 
	return mSolutionPath;
}

Vec2 RRT::randConfEven() {
	float randX = rand() % mWidth;
	float randY = rand() % mHeight;
	Vec2 myPos = Vec2(randX, randY);
	return myPos;
}

Vec2 RRT::randConfDist(Vec2 means, float sxx, float syy, float sxy) {
	normal_distribution<> dx{ means.x(), sxx };
	normal_distribution<> dy{ means.y(), syy };
	random_device rd{};
	mt19937 gen{ rd() };

	float a = dx(gen);
	float b = dy(gen);
	Vec2 sample = Vec2(a * sxx + b * sxy, a * sxy + b * syy);
	sample = toVec2(sample + means);

	return sample;
}

Node* RRT::nearestNode(Vec2 rand) {
	//loop through all points in the graph and find the one closest to the rand pos
	auto myNode = myTree->getNearestNode(rand);

	return myNode;
}

Node* RRT::cheapestNode(Vec2 rand) {
	//loop through all points nearby in the graph and find the cheapest one to the rand pos
	auto myNode = myTree->getCheapestNode(mRRTStarNeihborhood, rand);

	return myNode;
}

Vec2 RRT::newConf(Vec2 nearby, Vec2 rand) {
	//the new point is as far along the near-rand vector as you can
	//if we cant step that way at all, return a failed value (-1, -1)

	//is it the same point?
	Vec2 diffVec = toVec2(rand - nearby);
	if (diffVec.length() <= 1.0) {
		return Vec2(-1.0, -1.0);
	}

	// parameterization
	// circle: (x - center.x)^2 + (y - center.y)^2 = r^2
	// line: x = diffVec.x() * t + near.x()
	// line: y = diffVec.y() * t + near.y()
	// min possible t = 0 max possible t  = 1 for segment
	float tMin = 1;
	for (auto obstacle : mObstacles) {
		//what if the segment starts in the obstacle?
		if (toVec(obstacle.first - nearby).length() <= obstacle.second) {
			return Vec2(-1.0, -1.0);
		}

		float a = diffVec.length() * diffVec.length();
		float b = -2 * toVec(obstacle.first - nearby).dot(diffVec);
		float c = toVec(nearby - obstacle.first).length() * toVec(nearby - obstacle.first).length() - obstacle.second * obstacle.second;

		float d = b * b - 4 * a * c;
		if (d > 0.0) {
			//intersections possible
			float tPlus = (-b + sqrt(d)) / (2 * a);
			float tMinus = (-b - sqrt(d)) / (2 * a);

			if (tPlus <= 1 && tPlus >= 0) {
				//real intersection! 
				if (tPlus < tMin) {
					tMin = tPlus;
				}
			}

			if (tMinus <= 1 && tMinus >= 0) {
				//real intersection! 
				if (tMinus < tMin) {
					tMin = tMinus;
				}
			}
		}
	}

	auto res = (diffVec * tMin) + nearby;
	return toVec2(res);
}

bool RRT::intersects(Vec2 pos1, Vec2 pos2) {
	//is it the same point?
	Vec2 diffVec = toVec2(pos1 - pos2);
	if (diffVec.length() <= 1.0) {
		return false;
	}

	// parameterization
	// circle: (x - center.x)^2 + (y - center.y)^2 = r^2
	// line: x = diffVec.mX * t + near.mX
	// line: y = diffVec.mY * t + near.mY
	// min possible t = 0 max possible t  = 1 for segment
	float tMin = 1;
	for (auto obstacle : mObstacles) {
		//what if the segment starts in the obstacle?
		if (toVec(obstacle.first - pos1).length() <= obstacle.second) {
			return true;
		}

		float a = diffVec.length() * diffVec.length();
		float b = -2 * toVec(pos1 - obstacle.first).dot(diffVec);
		float c = toVec(pos1 - obstacle.first).length() * toVec(pos1 - obstacle.first).length() - obstacle.second * obstacle.second;

		float d = b * b - 4 * a * c;
		if (d > 0.0) {
			//intersections possible
			float tPlus = (-b + sqrt(d)) / (2 * a);
			float tMinus = (-b - sqrt(d)) / (2 * a);

			if (tPlus <= 1 && tPlus >= 0) {
				//real intersection! 
				if (tPlus < tMin) {
					return true;
				}
			}

			if (tMinus <= 1 && tMinus >= 0) {
				//real intersection! 
				if (tMinus < tMin) {
					return true;
				}
			}
		}
	}

	return false;
}

void RRT::letsBuildRRTStar() {
	Node* newNode = new Node(mInitPos, 0.f, nullptr);
	int count = 0;
	while (toVec(mGoalPos - newNode->mPosition).length() > 10.f) {
		if (count > mCountMax) break;
		count++;

		Vec2 randPos = Vec2(-1, -1);
		if (rand() % 100 < 10) {
			randPos = mGoalPos;
		}
		else {
			randPos = randConfEven();
		}

		Node* nearNode = cheapestNode(randPos);
		Vec2 tempNewPos = newConf(nearNode->mPosition, randPos);

		if (nearNode->mPosition.x() >= 0 && tempNewPos.x() > 0) {
			newNode = new Node(tempNewPos, toVec(tempNewPos - nearNode->mPosition).length(), nearNode);
			myTree->addVertex(newNode);
			myTree->addEdge(nearNode, newNode);
		}

		//rewire
		for (auto it : myTree->getList()) {
			if (toVec(it.second->mPosition - newNode->mPosition).length() < mRRTStarNeihborhood &&
				!intersects(it.second->mPosition, newNode->mPosition) && it.second->mParent != nullptr && it.second->mParent->mCost > newNode->mCost) {
				Node* oldParent = it.second->mParent;
				//remove me from the connected list
				oldParent->mConnectedNodes.erase(remove(oldParent->mConnectedNodes.begin(), oldParent->mConnectedNodes.end(), it.second), oldParent->mConnectedNodes.end());

				//change the parent
				it.second->mParent = newNode;
				newNode->mConnectedNodes.push_back(it.second);
			}
		}

	}
	mSolutionPath.clear();
	if (count > mCountMax) {
		cout << "Couldn't find a solution... So saaaad" << endl;
	} else {
		cout << "Found a solution! Yay go you!" << endl;
		mSolutionPath.push_back(newNode->mPosition);
		while (abs(newNode->mPosition.x() - mInitPos.x()) > 1.0 || abs(newNode->mPosition.y() - mInitPos.y()) > 1.0) {
			if (newNode->mParent == nullptr) {
				break;
			}
			newNode = newNode->mParent;
			mSolutionPath.push_back(newNode->mPosition);
		}
	}
}

void RRT::letsBuildRRT() {
	Node* newNode = new Node(mInitPos, 0.f, nullptr);
	int count = 0;
	while (toVec(mGoalPos - newNode->mPosition).length() > 10.f) {
		if (count > mCountMax) break;
		count++;

		Vec2 randPos = Vec2(-1, -1);
		if (rand() % 100 < 10) {
			randPos = mGoalPos;
		} else {
			randPos = randConfEven();
		}
		
		Node* nearNode = nearestNode(randPos);
		Vec2 tempNewPos = newConf(nearNode->mPosition, randPos);

		if (tempNewPos.x() > 0) {
			newNode = new Node(tempNewPos, toVec(tempNewPos - nearNode->mPosition).length(), nearNode);
			myTree->addVertex(newNode);
			myTree->addEdge(nearNode, newNode);
		}
	}
	mSolutionPath.clear();
	if (count > mCountMax) {
		cout << "Couldn't find a solution... So saaaad" << endl;
	} else {
		cout << "Found a solution! Yay go you!" << endl;
		mSolutionPath.push_back(newNode->mPosition);
		while (abs(newNode->mPosition.x() - mInitPos.x()) > 1.0 || abs(newNode->mPosition.y() - mInitPos.y()) > 1.0) {
			if (newNode->mParent == nullptr) {
				break;
			}
			newNode = newNode->mParent;
			mSolutionPath.push_back(newNode->mPosition);
		}
	}
}

void RRT::initEnvironment() {
	Node* initNode = new Node(mInitPos, 0.f, nullptr);
	if (myTree) {
		auto t = myTree;
		delete(t);
		myTree = nullptr;
	}
	myTree = new Tree();
	myTree->addVertex(initNode);
}


