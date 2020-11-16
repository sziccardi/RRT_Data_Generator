#include "data_utils.h"

std::size_t hash<Vec2>::operator()(const Vec2& k) const
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

Node::Node(Vec2 position, Node* parent) {
	mPosition = position;
	mParent = parent;
}

Node::~Node() {
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

void Node::addConnection(Node* newNode) {
	mConnectedNodes.push_back(newNode);
}


Tree::Tree() {
	myList = unordered_map<Vec2, Node*>();
}
Tree::~Tree() {
	for (auto it : myList) {
		delete(it.second);
	}
	myList.clear();
}

unordered_map<Vec2, Node*> Tree::getList() { return myList; }

Node* Tree::getNode(Vec2 pos) {
	return myList.at(pos);
}

void Tree::addVertex(Node* myNode) {
	if (!myNode) {
		cout << "You tried to add an empty node! So naughty.." << endl;
		return;
	}
	myList.insert(make_pair(myNode->mPosition, myNode));
}

void Tree::addEdge(Node* source, Node* destination) {
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

Node* Tree::getNearestNode(Vec2 pointC) {
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

int Tree::getTreeSize() { return myList.size(); }



	// Contructor which initialize the parameters.
Framework::Framework(int height_, int width_) : height(height_), width(width_) {
	SDL_Init(SDL_INIT_VIDEO);       // Initializing SDL as Video
	SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);      // setting draw color
	SDL_RenderClear(renderer);      // Clear the newly created window
	SDL_RenderPresent(renderer);    // Reflects the changes done in the
									//  window.
}

// Destructor
Framework::~Framework() {
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}

void Framework::draw_circle(int center_x, int center_y, int radius_, Vec3 color) {
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

void Framework::draw_tree(Tree* myTree, Vec3 color) {
	//draw tree
	SDL_SetRenderDrawColor(renderer, color.x() * 255, color.y() * 255, color.z() * 255, 255);

	auto myList = myTree->getList();
	for (auto realPair : myList) {
		for (Node* connectedNode : (realPair.second)->mConnectedNodes) {
			SDL_RenderDrawLine(renderer, toVec2(realPair.first).x(), toVec2(realPair.first).y(), toVec2(connectedNode->mPosition).x(), toVec2(connectedNode->mPosition).y());
		}
	}
}

void Framework::draw_solution(vector<Vec2> solution, Vec3 color) {
	SDL_SetRenderDrawColor(renderer, color.x() * 255, color.y() * 255, color.z() * 255, 255);
	Vec2 prevPoint = Vec2(-1, -1);
	for (auto val : solution) {
		if (prevPoint.x() >= 0) {
			SDL_RenderDrawLine(renderer, prevPoint.x(), prevPoint.y(), val.x(), val.y());
		}
		prevPoint = val;
	}
}

void Framework::draw_dist(float chiSquareVal, float meanX, float meanY, float sXX, float sYY, float sXY, Vec3 color) {
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
	SDL_SetRenderDrawColor(renderer, color.x() * 255, color.y() * 255, color.z() * 255, 75);

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

	double angle = atan2(eigenVecs(0, 1).real(), eigenVecs(0, 0).real());

	//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
	//if (angle < 0)
	//	angle += 6.28318530718 / 2.f;
	cout << angle << endl;
	//Conver to degrees instead of radians
	//angle = 180 * angle / 3.14159265359;

	//Calculate the size of the minor and major axes
	double halfmajoraxissize = sqrt(chiSquareVal * eigenVals(0).real());
	double halfminoraxissize = sqrt(chiSquareVal * eigenVals(1).real());

	SDL_RenderDrawLine(renderer, meanX, meanY, meanX + halfminoraxissize * eigenVecs(0, 1).real(), meanY + halfminoraxissize * eigenVecs(0, 0).real());
	SDL_RenderDrawLine(renderer, meanX, meanY, meanX + halfmajoraxissize * eigenVecs(1, 1).real(), meanY + halfmajoraxissize * eigenVecs(1, 0).real());

	// Drawing circle
	for (int x = meanX - halfmajoraxissize; x <= meanX + halfmajoraxissize; x++) {
		for (int y = meanY - halfmajoraxissize; y <= meanY + halfmajoraxissize; y++) {
			float horizTerm = (x - meanX) * cos(angle) - (y - meanY) * sin(angle);
			float vertTerm = (x - meanX) * sin(angle) + (y - meanY) * cos(angle);

			if (abs((std::pow(horizTerm / halfmajoraxissize, 2) + std::pow(vertTerm / halfminoraxissize, 2)) - 1) <=
				0.01f) {
				SDL_RenderDrawPoint(renderer, x, y);
			}
		}
	}

}

void Framework::present_render() {
	SDL_RenderPresent(renderer);
}

void Framework::save_img(const char* imgName) {
	SDL_Surface* sshot = SDL_CreateRGBSurface(0, 600, 600, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000);
	SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_ARGB8888, sshot->pixels, sshot->pitch);
	IMG_SavePNG(sshot, imgName);
	SDL_FreeSurface(sshot);
}

