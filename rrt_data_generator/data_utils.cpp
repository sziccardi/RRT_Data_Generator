#include "data_utils.h"

//////////////////////////////////////////////////////////////

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

Node::Node(Vec2 position, float newCost, Node* parent) {
	mPosition = position;
	mParent = parent;
	if (parent) mCost = newCost + parent->mCost;
	else mCost = newCost;
}

Node::~Node() {
		mParent = nullptr;
		mConnectedNodes.clear();
	}

void Node::addConnection(Node* newNode) {
	mConnectedNodes.push_back(newNode);
}

//////////////////////////////////////////////////////////////

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
	Node* nearest = new Node(Vec2(-1, -1), -1.f, nullptr);
	for (auto myPair : myList) {
		auto actualPos = Vec2(myPair.first.mData[0][0], myPair.first.mData[1][0]);
		float tempDelta = toVec2(actualPos - pointC).length();
		if (tempDelta < delta) {
			delta = tempDelta;
			if (nearest->mPosition.x() < 0) delete(nearest);
			nearest = myPair.second;
		}
	}
	return nearest;
}

Node* Tree::getCheapestNode(int neighborhoodRadius, Vec2 pointC) {
	float cost = 10000000000000.;
	Node* cheapest = new Node(Vec2(-1, -1), -1.f, nullptr);
	for (auto myPair : myList) {
		auto actualPos = Vec2(myPair.first.mData[0][0], myPair.first.mData[1][0]);
		float tempDelta = toVec2(actualPos - pointC).length();
		if (tempDelta < neighborhoodRadius && myPair.second->mCost < cost) {
			cost = myPair.second->mCost;
			if (cheapest->mPosition.x() < 0) delete(cheapest);
			cheapest = myPair.second;
		}
	}
	return cheapest;
}

int Tree::getTreeSize() { return myList.size(); }


