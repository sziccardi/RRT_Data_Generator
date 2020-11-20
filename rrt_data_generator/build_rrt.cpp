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
	} else {
		letsBuildRRTOnDist(means, sxx, syy, sxy);
	}
	return mSolutionPath;
}

void RRT::draw(Framework* fw, Vec3 solutionColor, Vec3 treeColor, bool drawObs, bool drawDist, Vec3 distColor, Vec2 means, float sxx, float syy, float sxy) {

	//draw the tree
	fw->draw_tree(myTree, treeColor);
	// Calling the function that draws circle.
	if (drawObs) {
		fw->draw_circle(mInitPos.x(), mInitPos.y(), 5, Vec3(255.f, 0.f, 0.f));
		fw->draw_circle(mGoalPos.x(), mGoalPos.y(), 5, Vec3(255.f, 0.f, 0.f));

		for (auto obs : mObstacles) {
			fw->draw_circle(obs.first.x(), obs.first.y(), obs.second, Vec3(0.f, 0.f, 255.f));
		}
	}
	//draw solution
	fw->draw_solution(mSolutionPath, solutionColor);

	if (drawDist) fw->draw_dist(5.f, means.x(), means.y(), sxx, syy, sxy, distColor);
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

//Vec2 nearestPixel(Vec2 rand) {
//  //loop through all points in the graph and find the one closest to the rand pos
//  Vec2 myPos = myTree.getNearestPoint(rand);

//  return myPos;
//}

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

void RRT::letsBuildRRTStarOnDist(Vec2 means, float sxx, float syy, float sxy) {

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
	}
	mSolutionPath.clear();
	if (count > mCountMax) {
		cout << "Couldn't find a solution... So saaaad" << endl;
	}
	else {
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
	}
	else {
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

void RRT::letsBuildRRTOnDist(Vec2 means, float sxx, float syy, float sxy) {
	Node* newNode = new Node(mInitPos, 0.f, nullptr);
	bool useDist = (means.x() > 0 && means.y() > 0);
	Eigen::Vector2f mean(2);
	mean(0) = means.x();
	mean(1) = means.y();
	Eigen::Matrix2f covar(2, 2);
	covar(0, 0) = sxx;
	covar(0, 1) = sxy;
	covar(1, 0) = sxy;
	covar(1, 1) = syy;
	Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
	int count = 0;
	while (toVec(mGoalPos - newNode->mPosition).length() > 10.f) {

		if (count > mCountMax) break;
		count++;

		Vec2 randPos = Vec2(-1, -1);
		if (means.x() > 0 && means.y() > 0) {
			if (rand() % 100 < 10) {
				randPos = mGoalPos;
			} else {
				auto sample = normX_solver1.samples(1);
				randPos = Vec2(sample(0, 0), sample(1, 0));
			}
			//cout << "chose " << randPos.x() << ", " << randPos.y() << endl;
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


