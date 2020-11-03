#include <cmath>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "matrices.h"

#include <SDL.h>
#include <SDL_Image.h>

#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/src/Cholesky/LLT.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "eigenmvn.h"

using namespace std;

namespace std {

	template <>
	struct hash<Vec2>
	{
		std::size_t operator()(const Vec2& k) const
		{
			using std::size_t;
			using std::hash;
			using std::string;

			// Compute individual hash values for first,
			// second and third and combine them using XOR
			// and bit shifting:

			return ((hash<float>()(k.mData[0][0])
				^ (hash<float>()(k.mData[1][0]) << 1)) >> 1);
		}
	};

}

class Node {
public:
	Node* mParent = nullptr;
	Vec2 mPosition = Vec2(-1, -1);
	vector<Node*> mConnectedNodes;

	Node(Vec2 position, Node* parent) {
		mPosition = position;
		mParent = parent;
	}
	~Node() {
		if (mParent) {
			auto p = mParent;
			delete(p);
			mParent = nullptr;
		}
		for (auto it = mConnectedNodes.begin(); it != mConnectedNodes.end(); ) {
			if (true) {
				delete * it;
				it = mConnectedNodes.erase(it);
			} else {
				++it;
			}
		}
		mConnectedNodes.clear();
	}

	void addConnection(Node* newNode) {
		mConnectedNodes.push_back(newNode);
	}

	/*bool operator==(const Node& m) {
		if (mParent == m.mParent &&
			mPosition.x() == toVec2(m.mPosition).x() &&
			mPosition.y() == toVec2(m.mPosition).y() &&
			mConnectedNodes.size() == m.mConnectedNodes.size()) {
			int count = 0;
			for (auto it : mConnectedNodes) {
				bool foundIt = false;
				for (auto cit : m.mConnectedNodes) {
					if (it == cit) {
						foundIt = true;
						break;
					}
				}
				if (foundIt) count++;
			}
			if (count == mConnectedNodes.size()) return true;
		}
		return false;
	}*/
};

class Tree {

private:
	unordered_map<Vec2, Node*> myList;

public:
	Tree() {
		myList = unordered_map<Vec2, Node*>();
	}
	~Tree() {
		for (auto it : myList) {
			delete(it.second);
		}
		myList.clear();
	}

	unordered_map<Vec2, Node*> getList() { return myList; }

	Node* getNode(Vec2 pos) {
		return myList.at(pos);
	}

	void addVertex(Node* myNode) {
		if (!myNode) {
			cout << "You tried to add an empty node! So naughty.." << endl;
			return;
		}
		myList.insert(make_pair(myNode->mPosition, myNode));
	}

	void addEdge(Node* source, Node* destination) {
		if (!source || !destination) {
			cout << "You tried to connnect empty nodes! So naughty.." << endl;
			return;
		}

		auto finding = myList.find(source->mPosition);
		if (finding == myList.end()) {
			cout << "Couldn't add that edge because its not from a real vertex." << endl;
			return;
		}
		finding = myList.find(destination->mPosition);
		if (finding != myList.end()) {
			addVertex(destination);
		}

		source->addConnection(destination);
	}

	Node* getNearestNode(Vec2 pointC) {
		float delta = 10000000000000.;
		Node* nearest = new Node(Vec2(-1, -1), nullptr);
		for (auto myPair : myList) {
			auto actualPos = Vec2(myPair.first.mData[0][0], myPair.first.mData[1][0]);
			float tempDelta = toVec2(actualPos - pointC).length();
			if (tempDelta < delta) {
				delta = tempDelta;
				nearest = myPair.second;
			}
		}
		return nearest;
	}

	int getTreeSize() { return myList.size(); }
};


class Framework {
public:
	// Contructor which initialize the parameters.
	Framework(int height_, int width_) : height(height_), width(width_) {
		SDL_Init(SDL_INIT_VIDEO);       // Initializing SDL as Video
		SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);      // setting draw color
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

	void draw_circle(int center_x, int center_y, int radius_, Vec3 color) {
		// Setting the color to be RED with 100% opaque (0% trasparent).
		SDL_SetRenderDrawColor(renderer, color.x(), color.y(), color.z(), 255);

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

	void draw_tree(Tree* myTree, Vec3 color) {
		//draw tree
		SDL_SetRenderDrawColor(renderer, color.x(), color.y(), color.z(), 175);

		auto myList = myTree->getList();
		for (auto realPair : myList) {
			for (Node* connectedNode : (realPair.second)->mConnectedNodes) {
				SDL_RenderDrawLine(renderer, toVec2(realPair.first).x(), toVec2(realPair.first).y(), toVec2(connectedNode->mPosition).x(), toVec2(connectedNode->mPosition).y());
			}
		}
	}

	void draw_solution(vector<Vec2> solution) {
		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		Vec2 prevPoint = Vec2(-1, -1);
		for (auto val : solution) {
			if (prevPoint.x() >= 0) {
				SDL_RenderDrawLine(renderer, prevPoint.x(), prevPoint.y(), val.x(), val.y());
			}
			prevPoint = val;
		}
	}

	void draw_dist(float chiSquareVal, float meanX, float meanY, float sXX, float sYY, float sXY, Vec3 color) {
		/*def plotErrorEllipseSigLvl(x_t, Sig_t, sig_lvl, conf_str, ax) :
			st_sig = Sig_t[:2, : 2]
			w, v = np.linalg.eig(st_sig)
			print(w, v)
			#w = abs(w) #TODO: Is this a hack ?
			w = np.clip(w, 0, 9e9)
			ang = math.atan2(v[1][0], v[0][0])
			ell = matplotlib.patches.Ellipse(xy = (x_t[0], x_t[1]), width = 2.0 * math.sqrt(sig_lvl * w[0]), height = 2.0 * math.sqrt(sig_lvl * w[1]), angle = np.rad2deg(ang), edgecolor = conf_str, fc = 'None', lw = 1)
			ax.add_patch(ell)*/

			// Setting the color to be RED with 100% opaque (0% trasparent).
		SDL_SetRenderDrawColor(renderer, color.x(), color.y(), color.z(), 255);

		Eigen::MatrixXd A(2, 2);
		A(0, 0) = sXX;
		A(0, 1) = sXY;
		A(1, 0) = sXY;
		A(1, 1) = sYY;

		Eigen::EigenSolver<Eigen::Matrix<double, 2, 2> > s(A);
		auto eigenVecs = s.eigenvectors();
		auto eigenVals = s.eigenvalues();

		Eigen::Vector2cd vec1 = Eigen::Vector2cd(eigenVecs(0, 1), eigenVecs(0, 0));
		Eigen::Vector2cd Vec2 = Eigen::Vector2cd(eigenVecs(1, 1), eigenVecs(1, 0));

		double angle = atan2(sqrt(eigenVecs(0, 1).real() * eigenVecs(0, 1).real()), sqrt(eigenVecs(0, 0).real() * eigenVecs(0, 0).real()));

		//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
		if (angle < 0)
			angle += 6.28318530718;

		//Conver to degrees instead of radians
		angle = 180 * angle / 3.14159265359;

		//Calculate the size of the minor and major axes
		double halfmajoraxissize = sqrt(chiSquareVal * eigenVals(0).real());
		double halfminoraxissize = sqrt(chiSquareVal * eigenVals(1).real());

		// Drawing circle
		for (int x = meanX - halfminoraxissize; x <= meanX + halfminoraxissize; x++) {
			for (int y = meanY - halfmajoraxissize; y <= meanY + halfminoraxissize; y++) {
				/*if ((std::pow(meanY - y, 2) + std::pow(meanX - x, 2)) <=
					std::pow(radius_, 2)) {
					SDL_RenderDrawPoint(renderer, x, y);
				}*/
			}
		}

	}

	void present_render() {
		SDL_RenderPresent(renderer);
	}

	void save_img(const char* imgName) {
		SDL_Surface* sshot = SDL_CreateRGBSurface(0, 600, 600, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000);
		SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_ARGB8888, sshot->pixels, sshot->pitch);
		IMG_SavePNG(sshot, imgName);
		SDL_FreeSurface(sshot);
	}

	/*bool operator==(const Framework& m) {
		if (height == m.height &&
			width == m.width &&
			renderer == m.renderer &&
			window == m.window) {
			return true;
		}
		return false;
	}*/

private:
	int height;     // Height of the window
	int width;      // Width of the window
	SDL_Renderer* renderer = NULL;      // Pointer for the renderer
	SDL_Window* window = NULL;      // Pointer for the window
};