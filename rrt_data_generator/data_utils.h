#pragma once
#include <cmath>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "matrices.h"

using namespace std;

namespace std {
	template <> struct hash<Vec2> {
		std::size_t operator()(const Vec2& k) const;
	};
}

class Node { 
public:
	Node(Vec2 position, float newCost, Node* parent);
	~Node();

	void addConnection(Node* newNode);

	Node* mParent = nullptr;
	Vec2 mPosition = Vec2(-1, -1);
	vector<Node*> mConnectedNodes;
	float mCost = -1.f;
};

class Tree {
private:
	unordered_map<Vec2, Node*> myList;

public:
	Tree();
	~Tree();

	Node* getNode(Vec2 pos);
	void addVertex(Node* myNode);
	void addEdge(Node* source, Node* destination);
	unordered_map<Vec2, Node*> getList();
	Node* getNearestNode(Vec2 pointC);
	Node* getCheapestNode(int neighborhoodRadius, Vec2 pointC);
	int getTreeSize();
};


