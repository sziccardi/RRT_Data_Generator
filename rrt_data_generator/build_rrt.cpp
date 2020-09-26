#include <cmath>
#include <vector>
#include <iostream>
#include <random>
#include <iomanip>

#include <objidl.h>

#include "data_utils.cpp"

#include <SDL.h>

using namespace std;

class Framework {
public:
	// Contructor which initialize the parameters.
	Framework(int height_, int width_) : height(height_), width(width_) {
		SDL_Init(SDL_INIT_VIDEO);       // Initializing SDL as Video
		SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);      // setting draw color
		SDL_RenderClear(renderer);      // Clear the newly created window
		SDL_RenderPresent(renderer);    // Reflects the changes done in the
										//  window.
	}

	// Destructor
	~Framework() {
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
	}

	void draw_circle(int center_x, int center_y, int radius_) {
		// Setting the color to be RED with 100% opaque (0% trasparent).
		SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

		// Drawing circle
		for (int x = center_x - radius_; x <= center_x + radius_; x++) {
			for (int y = center_y - radius_; y <= center_y + radius_; y++) {
				if ((std::pow(center_y - y, 2) + std::pow(center_x - x, 2)) <=
					std::pow(radius_, 2)) {
					SDL_RenderDrawPoint(renderer, x, y);
				}
			}
		}

		// Show the change on the screen
	}

	void draw_tree(Tree* myTree) {
		//draw tree
		SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);

		auto myList = myTree->getList();
		for (auto realPair : myList) {
			for (Node* connectedNode : (realPair.second)->mConnectedNodes) {
				SDL_RenderDrawLine(renderer, realPair.first.mX, realPair.first.mY, connectedNode->mPosition.mX, connectedNode->mPosition.mY);
			}
		}

		
	}

	void draw_solution(vector<vec2> solution) {
		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		vec2 prevPoint = vec2(-1, -1);
		for (auto val : solution) {
			if (prevPoint.mX >= 0) {
				SDL_RenderDrawLine(renderer, prevPoint.mX, prevPoint.mY, val.mX, val.mY);
			} 
			prevPoint = val;
		}
	}

	void present_render() {
		SDL_RenderPresent(renderer);
	}

private:
	int height;     // Height of the window
	int width;      // Width of the window
	SDL_Renderer* renderer = NULL;      // Pointer for the renderer
	SDL_Window* window = NULL;      // Pointer for the window
};

class RRT {
public:
	bool mUseDist = false;

	RRT() { initEnvironment(); }

	RRT(int confWidth, int confHeight, vec2 startPos, vec2 goalPos, int numVerts) {
		mWidth = confWidth;
		mHeight = confHeight;

		mInitPos = startPos;
		mGoalPos = goalPos;

		mNumVertices = numVerts;
		initEnvironment();
		return;
	}

	vec2 getInitPos() { return mInitPos; }
	void setInitPos(vec2 initPos) { mInitPos = initPos; }

	vec2 getGoalPos() { return mGoalPos; }
	void setGoalPos(vec2 goalPos) { mGoalPos = goalPos; }

	float getConfWidth() { return mWidth; }
	float getConfHeight() { return mHeight; }
	void setConfSize(float width, float height) { mWidth = width; mHeight = height; }

	int getNumVertices() { return mNumVertices; }
	void setNumVertices(int numVerts) { mNumVertices = numVerts; }
	
	int getNumNodes() { return myTree->getTreeSize(); }

	void addObstacle(vec2 pos, float radius) {
		mObstacles.push_back(make_pair(pos, radius));
	}

	vector<vec2> start() {
		letsBuildRRT();
		return mSolutionPath;
	}

	vector<vec2> start(vec2 means, float sxx, float syy, float sxy) {
		letsBuildRRT(means, sxx, syy, sxy);
		return mSolutionPath;
	}

	void draw() {

		// Creating the object by passing Height and Width value.
		Framework fw(600, 600);

		//draw the tree
		fw.draw_tree(myTree);
		// Calling the function that draws circle.
		fw.draw_circle(mInitPos.mX, mInitPos.mY, 5);
		fw.draw_circle(mGoalPos.mX, mGoalPos.mY, 5);

		for (auto obs : mObstacles) {
			fw.draw_circle(obs.first.mX, obs.first.mY, obs.second);
		}

		//draw solution
		fw.draw_solution(mSolutionPath);
		//show
		fw.present_render();

		

		SDL_Event event = SDL_Event();    // Event variable

		// Below while loop checks if the window has terminated using close in the
		//  corner.
		while (!(event.type == SDL_QUIT)) {
			SDL_Delay(10);  // setting some Delay
			SDL_PollEvent(&event);  // Catching the poll event.
		}

	}

private:
	int mNumVertices = 154;
	Tree* myTree;
	vector<vec2> mSolutionPath;
	vec2 mInitPos = vec2(-1.0, -1.0);
	vec2 mGoalPos = vec2(-1.0, -1.0);
	vector<pair<vec2, float>> mObstacles;
	float dq = 10;
	int mWidth = 0;
	int mHeight = 0;

	vec2 randConfEven() {
		float randX = rand() % mWidth;
		float randY = rand() % mHeight;
		vec2 myPos = vec2(randX, randY);
		return myPos;
	}

	vec2 randConfDist(vec2 means = vec2(-1, -1), float sxx = -1, float syy = -1, float sxy = -1) {
		normal_distribution<> dx{ means.mX, sxx };
		normal_distribution<> dy{ means.mY, syy };
		random_device rd{};
		mt19937 gen{ rd() };

		float a = dx(gen);
		float b = dy(gen);
		vec2 sample = vec2(a*sxx + b*sxy, a*sxy + b*syy);
		sample = sample + means;

		return sample;
	}

	//vec2 nearestPixel(vec2 rand) {
	//  //loop through all points in the graph and find the one closest to the rand pos
	//  vec2 myPos = myTree.getNearestPoint(rand);

	//  return myPos;
	//}

	Node* nearestNode(vec2 rand) {
		//loop through all points in the graph and find the one closest to the rand pos
		auto myNode = myTree->getNearestNode(rand);

		return myNode;
	}

	vec2 newConf(vec2 nearby, vec2 rand) {
		//the new point is as far along the near-rand vector as you can
		//if we cant step that way at all, return a failed value (-1, -1)

		//is it the same point?
		vec2 diffVec = (rand - nearby);
		if (diffVec.vecLength() <= 1.0) {
			return vec2(-1.0, -1.0);
		}

		// parameterization
		// circle: (x - center.x)^2 + (y - center.y)^2 = r^2
		// line: x = diffVec.mX * t + near.mX
		// line: y = diffVec.mY * t + near.mY
		// min possible t = 0 max possible t  = 1 for segment
		float tMin = 1;
		for (auto obstacle : mObstacles) {
			//what if the segment starts in the obstacle?
			if ((obstacle.first - nearby).vecLength() <= obstacle.second) {
				return vec2(-1.0, -1.0);
			}

			float a = diffVec.vecLength() * diffVec.vecLength();
			float b = -2 * (obstacle.first - nearby).dotProduct(diffVec);
			float c = (nearby - obstacle.first).vecLength() * (nearby - obstacle.first).vecLength() - obstacle.second * obstacle.second;

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
		return (res);
	}

	void letsBuildRRT(vec2 means = vec2(-1, -1), float sxx = -1, float syy = -1, float sxy = -1) {
		Node* newNode = new Node(mInitPos, nullptr);
		while ((mGoalPos - newNode->mPosition).vecLength() > 10.f) {
			vec2 randPos = vec2(-1, -1);
			if (mUseDist) {
				randPos = randConfDist(means, sxx, syy, sxy);
			}
			else {
				randPos = randConfEven();
			}

			Node* nearNode = nearestNode(randPos);
			vec2 tempNewPos = newConf(nearNode->mPosition, randPos);

			if (tempNewPos.mX > 0) {
				newNode = new Node(tempNewPos, nearNode);
				myTree->addVertex(newNode);
				myTree->addEdge(nearNode, newNode);
			}
		}
		cout << "Found a solution! Yay go you!" << endl;
		mSolutionPath.clear();
		mSolutionPath.push_back(newNode->mPosition);
		while (abs(newNode->mPosition.mX - mInitPos.mX) > 1.0 || abs(newNode->mPosition.mY - mInitPos.mY) > 1.0) {
			if (newNode->mParent == nullptr) {
				break;
			}
			newNode = newNode->mParent;
			mSolutionPath.push_back(newNode->mPosition);
		}
	}

	/*void drawSolutionPath() {
		vec2 oldPos = vec2(-1.0, -1.0);
		for (Node* node : mSolutionPath) {
			if (oldPos.mX > 0) {
				fill(255, 0, 0);
				strokeWeight(5);
				line(oldPos.mX, oldPos.mY, pos.mPosition.mX, pos.mPosition.mY);
			}
			oldPos = node->mPosition;
		}
		line(oldPos.mX, oldPos.mY, mInitPos.mX, mInitPos.mY);
		strokeWeight(1);
	}*/

	/*void drawObstacles() {
		for (vec2 obstacle : mObstacles) {
			fill(255, 50);
			ellipse(obstacle.mX, obstacle.mY, mObstacleRadius * 2, mObstacleRadius * 2);
		}
	}*/

	void initEnvironment() {
		//mInitPos = new vec2(0.0, height / 2.0);
		//mGoalPos = new vec2(width / 1.0, height / 2.0);
		Node* initNode = new Node(mInitPos, nullptr);
		if (myTree) {
			auto t = myTree;
			delete(t);
			myTree = nullptr;
		}
		myTree = new Tree();
		myTree->addVertex(initNode);
		/*vec2 randPos; = randConf();
		vec2 newPos = vec2(-1.0, -1.0);
		while (newPos.mX < 0) {
			newPos = newConf(mInitPos, randPos);
		}
		Node* newNode = new Node(newPos, initNode);
		myTree->addVertex(newNode);
		myTree->addEdge(initNode, newNode);*/
	}

	//void draw() {
	//	background(51);

	//	//draw start button
	//	fill(0);
	//	rect(20, 20, 20, 20);

	//	if (mInitPos.mX >= 0) {
	//		//draw start place
	//		fill(255, 0, 0);
	//		ellipse(mInitPos.mX, mInitPos.mY, 15, 15);
	//	}
	//	if (mGoalPos.mX >= 0) {
	//		//draw end place
	//		fill(255, 0, 0);
	//		ellipse(mGoalPos.mX, mGoalPos.mY, 15, 15);
	//	}
	//	myTree.drawTree();
	//	drawObstacles();
	//	drawSolutionPath();
	//}

	//void mouseClicked() {
	//	if (mouseX > 20 && mouseX < 40 && mouseY > 20 && mouseY < 40) {
	//		print("STARTING\n");
	//		initEnvironment();
	//		letsBuildRRT();
	//	} else {
	//		//place obstacles 
	//		print("PLACING OBSTACLE\n");
	//		vec2 newThing = new vec2((float)mouseX, (float)mouseY);
	//		boolean canPlace = true;
	//		for (vec2 obstacle : mObstacles) {
	//			if ((newThing.vecSubtract(obstacle)).vecvecLength() < 2) {
	//				canPlace = false;
	//				break;
	//			}
	//		}
	//		if (canPlace) {
	//			mObstacles.add(newThing);
	//		}
	//	}
	//}
};

