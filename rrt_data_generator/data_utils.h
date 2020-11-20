#pragma once
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

class Framework {
public:
	Framework(int height_, int width_);
	~Framework();

	void draw_circle(int center_x, int center_y, int radius_, Vec3 color);
	void draw_tree(Tree* myTree, Vec3 color);
	void draw_solution(vector<Vec2> solution, Vec3 color);
	void draw_dist(float chiSquareVal, float meanX, float meanY, float sXX, float sYY, float sXY, Vec3 color);
	void present_render();
	void save_img(const char* imgName);

private:
	int height;     // Height of the window
	int width;      // Width of the window
	SDL_Renderer* renderer = NULL;      // Pointer for the renderer
	SDL_Window* window = NULL;      // Pointer for the window
};
