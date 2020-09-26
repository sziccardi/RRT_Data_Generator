
#include "matrices.h"

//default initialiation (mData is empty) possibly dangerous?
Matrix::Matrix() {

}

//initialize with zeros
Matrix::Matrix(int width, int height) {
	mNumCols = width;
	mNumRows = height;
	mData.resize((size_t)height);
	for (int i = 0; i < height; i++) {
		mData[i].resize(width, 0.f);
	}
	mInitialized = true;
}

//initialize with data
Matrix::Matrix(int width, int height, std::vector<std::vector<float>> data) {
	mNumCols = width;
	mNumRows = height;
	mData.resize((size_t)height);
	for (int i = 0; i < height; i++) {
		mData[i].resize(width, 0.f);
		for (int j = 0; j < width; j++) {
			mData[i][j] = data[i][j];
		}
	}
	mInitialized = true;
}

//deconstructor
Matrix::~Matrix() {
	for (int i = 0; i < mNumRows; i++) {
		mData[i].clear();
	}
	mData.clear();
	mInitialized = false;
}

string Matrix::toString() {
	//  | X  X  X  X  X  |
	//  | X  X  X  X  X  |
	//  | X  X  X  X  X  |
	//  | X  X  X  X  X  |
	//  | X  X  X  X  X  |
	string result = "";
	if (mInitialized) {
		for (int i = 0; i < mNumRows; i++) {
			result += "| ";
			for (int j = 0; j < mNumCols; j++) {
				result += to_string(mData[i][j]) + " ";
			}

			result += "|\n";
		}
	} else {
		cout << "YOU'RE TRYING TO PRINT A MATRIX THATS NOT INITIALIZED! YOU MESSED UP SOMEWHERE" << endl;
	}
	return result;
}

//getters and setters
float Matrix::at(int i, int j) {
	if (i < 0 || i >= mNumRows || j < 0 || j >= mNumCols) {
		cout << "TRYING TO ACCESS VALUE OUTSIDE LIMIT. ACCESSING (" << to_string(i) << ", " << to_string(j) << ") WHEN THE MATRIX IS ONLY " << to_string(mNumRows) << "x" << to_string(mNumCols) << endl;
		return INT_MAX;
	}
	return mData[i][j];
}

int Matrix::width() {
	return mNumCols;
}

int Matrix::height() {
	return mNumRows;
}

int Matrix::numRows() {
	return mNumRows;
}

int Matrix::numCols() {
	return mNumCols;
}

void Matrix::setVal(int i, int j, float val) {
	if (i < mNumRows && j < mNumCols) {
		mData[i][j] = val;
	} else {
		cout << "YOU TRIED TO SET A VALUE OUTSIDE THE DEFINED MATRIX" << endl;
	}
}

//operators

Matrix Matrix::operator+(const Matrix& m) {
	Matrix temp = plus(m);
	return temp;
}

void Matrix::operator+=(const Matrix& m) {
	add(m);
}

Matrix Matrix::operator-(const Matrix& m) {
	Matrix temp = minus(m);
	return temp;
}

void Matrix::operator-=(const Matrix& m) {
	subtract(m);
}

Matrix Matrix::operator*(const float& s) {
	auto temp = times(s);
	return temp;
}

Matrix Matrix::operator*(Matrix & m) {
	auto temp = times(m);
	return temp;
}

//Vec Matrix::operator*(Vec & v) {
//	auto tempM = times(v);
//	Vec tempV = Vec();
//	return tempV.toVec(tempM);
//}

void Matrix::operator*=(const float& s) {
	mult(s);
}

void Matrix::operator*=(Matrix& m) {
	mult(m);
}

bool Matrix::operator==(const Matrix& m) {
	if (mNumRows == m.mNumRows && mNumCols == m.mNumCols) {
		for (int i = 0; i < mNumRows; i++) {
			for (int j = 0; j < mNumCols; j++) {
				if (mData[i][j] != m.mData[i][j]) {
					return false;
				}
			}
		}
	} else {
		cout << "COULD NOT EQUATE MATRICES, SIZE MISMATCH" << endl;
		return false;
	}
	return true;
}

//matrix operations
///Trace
float Matrix::Trace() {
	if (mNumCols == mNumRows) {
		float total = 0;
		for (int i = 0; i < mNumCols; i++) {
			total += mData[i][i];
		}
		return total;
	} else {
		cout << "TRACE UNDEFINED, NON-SQUARE MATRIX" << endl;
		return INT_MAX;
	}
}
///return new matrix that is the transpose
Matrix Matrix::Transpose() {
	Matrix temp = Matrix(mNumCols, mNumRows);
	for (int i = 0; i < mNumRows; i++) {
		for (int j = 0; j < mNumCols; j++) {
			temp.mData[j][i] = mData[i][j];
		}
	}
	return temp;
}
///transpose current matrix
void Matrix::Transposed() {
	Matrix temp = Matrix(mNumCols, mNumRows, mData);
	for (int i = 0; i < mNumRows; i++) {
		for (int j = 0; j < mNumCols; j++) {
			mData[j][i] = temp.mData[i][j];
		}
	}
}

///return new matrix with the mat as row reduced as possible
Matrix Matrix::ReduceRows() {
	Matrix temp = Matrix(mNumCols, mNumRows, mData);

	//row to start reduction at
	int rowToStartReduction = 0;

	//initialize left most non zero col (one to set to zero)
	int leftMostNonZeroCol = 0;
	bool foundNonZero = false;
	while (!foundNonZero) {
		bool allZero = true;
		for (int i = 0; i < mNumRows; i++) {
			if (temp.mData[i][leftMostNonZeroCol] >= 0.000000000001f) {//basically non-zero
				allZero = false;
			}
		}
		if (allZero) leftMostNonZeroCol++;
		else foundNonZero = true;
	}
	///FOR rows starting from rowToReduce
	for (int k = rowToStartReduction; k < mNumRows; k++) {
		//get 1 in the top left
		float scale = temp.mData[k][leftMostNonZeroCol];
		if (abs(temp.mData[k][leftMostNonZeroCol]) >= 0.000000000001f) {
			for (int j = k; j < mNumCols; j++) {
				temp.mData[k][j] /= scale;
			}
		} else {
			//TODO gotta get a 1 there some other way
		}

		//get zeros below
		for (int i = k + 1; i < mNumRows; i++) {
			float scale = temp.mData[i][leftMostNonZeroCol] * -1.f;
			if (abs(scale) >= 0.000000000001f) {
				for (int j = leftMostNonZeroCol; j < mNumCols; j++) {
					temp.mData[i][leftMostNonZeroCol] += temp.mData[k][j] * scale;
				}
			}
		}

		leftMostNonZeroCol++;
	}
	return temp;
}

void Matrix::ReducedRows() {
	///row to start reduction at
	int rowToStartReduction = 0;

	//initialize left most non zero col (one to set to zero)
	int leftMostNonZeroCol = 0;
	bool foundNonZero = false;
	while (!foundNonZero) {
		bool allZero = true;
		for (int i = 0; i < mNumRows; i++) {
			if (mData[i][leftMostNonZeroCol] >= 0.000000000001f) {//basically non-zero
				allZero = false;
			}
		}
		if (allZero) leftMostNonZeroCol++;
		else foundNonZero = true;
	}
		
	for (int k = rowToStartReduction; k < mNumRows; k++) {
		//get 1 in the top left
		float scale = mData[k][leftMostNonZeroCol];
		if (abs(mData[k][leftMostNonZeroCol]) >= 0.000000000001f) {
			for (int j = k; j < mNumCols; j++) {
				mData[k][j] /= scale;
			}
		} else {
			//TODO gotta get a 1 there some other way
		}

		//get zeros below
		for (int i = k + 1; i < mNumRows; i++) {
			float scale = mData[i][leftMostNonZeroCol] * -1.f;
			if (abs(scale) >= 0.000000000001f) {
				for (int j = leftMostNonZeroCol; j < mNumCols; j++) {
					mData[i][leftMostNonZeroCol] += mData[k][j] * scale;
				}
			}
		}

		leftMostNonZeroCol++;
	}
}

///return new matrix with the mat as column reduced as possible
//Matrix ReduceCols() {
		
//}

///rank : if skinny, column rank. if fat, row rank
int Matrix::Rank() {
	if (mNumCols >= mNumRows) {
		//fat matrix
		Matrix red = ReduceRows();
		int count = 0;
		for (int i = 0; i < mNumRows; i++) {
			bool allZero = true;
			for (int j = 0; j < mNumCols; j++) {
				if (abs(red.mData[i][j]) > 0.00000000001f) {
					allZero = false;
					break;
				}
			}
			if (!allZero) count++;
		}
	} else {
		//skinny matrix
	}

	return 0;
}

void Matrix::mult(float s) {
	for (int i = 0; i < mNumRows; i++) {
		for (int j = 0; j < mNumCols; j++) {
			mData[i][j] *= s;
		}
	}
}

Matrix& Matrix::times(float s) {
	Matrix* temp = new Matrix(mNumCols, mNumRows, mData);
	for (int i = 0; i < mNumRows; i++) {
		for (int j = 0; j < mNumCols; j++) {
			temp->mData[i][j] *= s;
		}
	}
	return (*temp);
}

void Matrix::mult(Matrix & m) {
	if (width() == m.height()) {
		Matrix* temp = new Matrix(m.mNumCols, mNumRows, mData);
		mData.clear();
		mData.resize(m.height());
		for (int i = 0; i < m.height(); i++) {
			mData[i].resize(mNumCols, 0.f);
			for (int j = 0; j < mNumCols; j++) {
				float sum = 0;
				for (int k = 0; k < width(); k++) {
					sum += temp->mData[i][k] * m.mData[k][j];
				}
				setVal(i, j, sum);
			}
		}
	} else {
		cout << "CAN'T MULTIPLY THESE MATRICES TOGETHER" << endl;
	}
}

Matrix & Matrix::times(Matrix & m) {
	if (width() == m.height()) {
		Matrix* temp = new Matrix(m.width(), mNumRows, mData);
		for (int i = 0; i < mNumRows; i++) {
			for (int j = 0; j < m.width(); j++) {
				float sum = 0;
				for (int k = 0; k < width(); k++) {
					sum += mData[i][k] * m.mData[k][j];
				}
				temp->setVal(i, j, sum);
			}
		}
		return (*temp);
	} else {
		cout << "CAN'T MULTIPLY THESE MATRICES TOGETHER" << endl;
		Matrix* emptyThing = new Matrix();
		return (*emptyThing);
	}
}

void Matrix::add(const Matrix& m) {
	if (mNumRows == m.mNumRows && mNumCols == m.mNumCols) {
		for (int i = 0; i < mNumRows; i++) {
			for (int j = 0; j < mNumCols; j++) {
				mData[i][j] += m.mData[i][j];
			}
		}
	} else {
		cout << "COULD NOT ADD MATRICES, SIZE MISMATCH" << endl;
	}
}

Matrix& Matrix::plus(const Matrix& m) {
	Matrix* temp = new Matrix(mNumCols, mNumRows, mData);
	if (mNumRows == m.mNumRows && mNumCols == m.mNumCols) {
		for (int i = 0; i < mNumRows; i++) {
			for (int j = 0; j < mNumCols; j++) {
				temp->mData[i][j] += m.mData[i][j];
			}
		}
	} else {
		cout << "COULD NOT ADD MATRICES, SIZE MISMATCH" << endl;
	}
	return (*temp);
}

void Matrix::subtract(const Matrix& m) {
	if (mNumRows == m.mNumRows && mNumCols == m.mNumCols) {
		for (int i = 0; i < mNumRows; i++) {
			for (int j = 0; j < mNumCols; j++) {
				mData[i][j] -= m.mData[i][j];
			}
		}
	} else {
		cout << "COULD NOT SUBTRACT MATRICES, SIZE MISMATCH" << endl;
	}
}

Matrix& Matrix::minus(const Matrix& m) {
	Matrix* temp = new Matrix(mNumCols, mNumRows, mData);
	if (mNumRows == m.mNumRows && mNumCols == m.mNumCols) {
		for (int i = 0; i < mNumRows; i++) {
			for (int j = 0; j < mNumCols; j++) {
				temp->mData[i][j] -= m.mData[i][j];
			}
		}
	} else {
		cout << "COULD NOT SUBTRACT MATRICES, SIZE MISMATCH" << endl;
	}
	return (*temp);
}

Identity::Identity(int size) : Matrix(size, size) {
	for (int i = 0; i < size; i++) {
		mData[i][i] = 1.f;
	}
}

Vec::Vec() : Matrix() { ; }

Vec::Vec(int size) : Matrix(1, size) {
	;
}

Vec::Vec(int size, std::vector<float> data) : Matrix(1, size) {
	for (int i = 0; i < size; i++) {
		mData[i][0] = data[i];
	}
}

Vec toVec(Matrix m) {
	if (m.width() > 1) {
		cout << "CANNOT TURN THIS MATRIX INTO A VECTOR, IT HAS TOO MANY COLUMNS" << endl;
		return Vec();
	}
	std::vector<float> data;
	for (int i = 0; i < m.height(); i++) {
		auto toAdd = m.at(i, 0);
		data.push_back(toAdd);
	}

	return Vec(m.height(), data);
}

string Vec::toString() {
	string result = "( ";
	if (mInitialized) {
		for (int i = 0; i < dims() - 1; i++) {
			result += to_string(at(i)) + ", ";
		}
		if (dims() > 0) result += to_string(at(dims() - 1));
	} else {
		cout << " YOU'RE TRYING TO PRINT A VECTOR THATS NOT INITIALIZED! YOU MESSED UP SOMEWHERE" << endl;
	}
	return result + " )\n";
}

int Vec::dims() {
	return height();
}

float Vec::at(int i) {
	return Matrix::at(i, 0);
}

//Return the length of this vector
float Vec::length() {
	float sum = 0;
	for (int i = 0; i < mNumRows; i++) {
		sum += mData[i][0] * mData[i][0];
	}

	return sqrt(sum);
}

//Return the square of the length of this vector
float Vec::lengthSqr() {
	float l = length();
	return l*l;
}

//Rescale this vector to have a unit length
void Vec::normalize() {
	mult(1 / length());
}

//Return a new vector in the same direction as this one, but with unit length
Vec Vec::normalized() {
	return toVec(times(1 / length()));
}

//If the vector is longer than maxL, shrink it to be maxL otherwise do nothing
void Vec::clampToLength(float maxL) {
	if (length() > maxL) {
		mult(maxL / length());
	}
}

//Grow or shrink the vector have a length of maxL
void Vec::setToLength(float newL) {
	mult(newL / length());
}

//Interepret the two vectors (this and the rhs) as points. How far away are they?
float Vec::distanceTo(Vec rhs) {
	if (dims() != rhs.dims()) {
		cout << "CANNOT FIND THE DISTANCE BETWEEN THINGS OF DIFFERENT DIMENTIONS!" << endl;
		return INT_MAX;
	}
	float sum = 0;
	for (int i = 0; i < mNumRows; i++) {
		float d = at(i) - rhs.at(i);
		sum += d * d;
	}

	return sqrt(sum);
}


Vec Vec::interpolate(Vec b, float t) {
	return toVec(plus((b.plus(times(-1))) * t));
}

float Vec::dot(Vec b) {
	if (dims() != b.dims()) {
		cout << "CANNOT DOT THINGS OF DIFFERENT DIMENTIONS!" << endl;
		return INT_MAX;
	}
	float sum = 0;
	for (int i = 0; i < mNumRows; i++) {
		sum += at(i) * b.at(i);
	}
	return sum;
}

Vec Vec::projAB(Vec b) {
	return toVec(b * (dot(b)));
}


Vec2::Vec2() : Vec(2) {

}

Vec2::Vec2(float x, float y) : Vec(2) {
	mData[0][0] = x;
	mData[1][0] = y;
}

Vec2 toVec2(Vec v) {
	if (v.dims() > 2) {
		cout << "CANNOT CAST THIS TO A VEC2, THERE ARE TOO MANY VALUES" << endl;
		return Vec2();
	}

	return Vec2(v.at(0), v.at(1));
}

Vec2 toVec2(Matrix m) {
	auto vecThing = toVec(m);
	return toVec2(vecThing);
}

float Vec2::x() {
	return mData[0][0];
}

float Vec2::y() {
	return mData[1][0];
}


Vec3::Vec3() : Vec(3) {}

Vec3::Vec3(float x, float y, float z) : Vec(3) {

	mData[0][0] = x;
	mData[1][0] = y;
	mData[2][0] = z;
}

Vec3 toVec3(Vec v) {
	if (v.dims() > 3) {
		cout << "CANNOT CAST THIS TO A VEC2, THERE ARE TOO MANY VALUES" << endl;
		return Vec3();
	}

	return Vec3(v.at(0), v.at(1), v.at(2));
}

Vec3 toVec3(Matrix m) {
	auto vecThing = toVec(m);
	return toVec3(vecThing);
}

float Vec3::x() {
	return mData[0][0];
}

float Vec3::y() {
	return mData[1][0];
}

float Vec3::z() {
	return mData[2][0];
}

Vec3& Vec3::cross(Vec3 secondVec) {
	float newX = mData[1][0] * secondVec.z() - mData[2][0] * secondVec.y();
	float newY = mData[2][0] * secondVec.y() - mData[0][0] * secondVec.z();
	float newZ = mData[0][0] * secondVec.x() - mData[1][0] * secondVec.x();

	return *(new Vec3(newX, newY, newZ));
}