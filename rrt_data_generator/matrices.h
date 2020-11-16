#pragma once
#include <cmath>
#include <vector>
#include <iostream>

#include <string>

using namespace std;

class Matrix {
public:
	Matrix();
	Matrix(int w, int h);
	Matrix(int w, int h, std::vector<std::vector<float>> d);

	~Matrix();

	string toString();
	string getType() {
		return "Matrix";
	}
	//getters/setters
	float at(int i, int j);
	int width();
	int height();
	int numRows();
	int numCols();
	bool isInitialized() { return mInitialized; }

	void setVal(int i, int j, float val);

	//operator overloading

	Matrix operator+(const Matrix& m);
	void operator+=(const Matrix& m);
	Matrix operator-(const Matrix& m);
	void operator-=(const Matrix& m);
	Matrix operator*(const float& s);
	Matrix operator*(Matrix& m);
	//Vec operator*(Vec& v);
	void operator*=(const float& s);
	void operator*=(Matrix& m);
	bool operator==(const Matrix& m);

	//matrix operations
	float Trace();
	Matrix Transpose();
	void Transposed();
	Matrix ReduceRows();
	void ReducedRows();
	int Rank();

	std::vector<std::vector<float>> mData;
protected:
	int mNumRows = -1;
	int mNumCols = -1;

	bool mInitialized = false;


	void mult(float s);
	Matrix& times(float s);
	void mult(Matrix& m);
	Matrix& times(Matrix& m);
	void add(const Matrix& m);
	Matrix& plus(const Matrix& m);
	void subtract(const Matrix& m);
	Matrix& minus(const Matrix& m);

};

class Identity : public Matrix {
public:
	Identity(int size);

	string getType() {
		return "Identity Matrix";
	}
};

class Vec : public Matrix {
public:
	Vec();
	Vec(int size);
	Vec(int size, std::vector<float> data);

	string toString();
	string getType() {
		return "Vector";
	}

	int dims();
	float at(int i);

	float length();
	float lengthSqr();
	void normalize();
	Vec normalized();
	void clampToLength(float maxL);
	void setToLength(float newL);
	float distanceTo(Vec rhs);
	Vec interpolate(Vec b, float t);
	float dot(Vec b);
	Vec projAB(Vec b);
};

class Vec3 : public Vec {
public:
	Vec3();
	Vec3(float x, float y, float z);

	float x();
	float y();
	float z();

	Vec3& cross(Vec3 secondVec);

	string getType() {
		return "Vec3";
	}
};


class Vec2 : public Vec {
public:
	Vec2();
	Vec2(float x, float y);

	float x();
	float y();

	string getType() {
		return "Vec2";
	}
	bool operator==(const Vec2& m) const;
};

//Conversions
Vec toVec(Matrix m);

Vec2 toVec2(Vec v);
Vec2 toVec2(Matrix m);

Vec3 toVec3(Vec v);
Vec3 toVec3(Matrix m);