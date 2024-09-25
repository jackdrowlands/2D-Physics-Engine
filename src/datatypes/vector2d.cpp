#include "../../include/datatypes/vector2d.hpp"

// Default constructor initializes vector to (0,0)
vector2d::vector2d() : x(0), y(0) {}

// Constructor initializes vector with given x and y values
vector2d::vector2d(double x, double y) : x(x), y(y) {}

// Constructor initializes vector with both x and y set to the given scalar
// value
vector2d::vector2d(double scalar) : x(scalar), y(scalar) {}

// Operator overloads for vector addition, subtraction, multiplication, and
// division
vector2d vector2d::operator+(const vector2d& other) const {
  return vector2d(x + other.x, y + other.y);
}

vector2d vector2d::operator-(const vector2d& other) const {
  return vector2d(x - other.x, y - other.y);
}

vector2d vector2d::operator*(const double& scalar) const {
  return vector2d(x * scalar, y * scalar);
}

vector2d vector2d::operator*(const vector2d& other) const {
  return vector2d(x * other.x, y * other.y);
}

vector2d vector2d::operator/(const double& scalar) const {
  return vector2d(x / scalar, y / scalar);
}

// Operator overloads for in-place vector addition, subtraction, multiplication,
// and division
vector2d& vector2d::operator+=(const vector2d& other) {
  x += other.x;
  y += other.y;
  return *this;
}

vector2d& vector2d::operator-=(const vector2d& other) {
  x -= other.x;
  y -= other.y;
  return *this;
}

vector2d& vector2d::operator*=(const double& scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}

vector2d& vector2d::operator*=(const vector2d& other) {
  x *= other.x;
  y *= other.y;
  return *this;
}

vector2d& vector2d::operator/=(const double& scalar) {
  x /= scalar;
  y /= scalar;
  return *this;
}

// Operator overloads for vector equality and inequality checks
bool vector2d::operator==(const vector2d& other) const {
  return x == other.x && y == other.y;
}

bool vector2d::operator!=(const vector2d& other) const {
  return x != other.x || y != other.y;
}

// Operator overload for accessing vector components by index
double& vector2d::operator[](int index) { return index == 0 ? x : y; }

const double& vector2d::operator[](int index) const {
  return index == 0 ? x : y;
}

// Operator overload for unary minus (negation)
vector2d vector2d::operator-() const { return vector2d(-x, -y); }

// Method to compute the magnitude (length) of the vector
double vector2d::magnitude() const { return std::sqrt(x * x + y * y); }

// Method to normalize the vector (make it have length 1)
vector2d vector2d::normalise() const {
  double mag = magnitude();
  return mag == 0 ? vector2d(0, 0) : vector2d(x / mag, y / mag);
}

// Method to compute the dot product of this vector with another
double vector2d::dot(const vector2d& other) const {
  return x * other.x + y * other.y;
}

// Method to compute the cross product of this vector with another
double vector2d::cross(const vector2d& other) const {
  return x * other.y - y * other.x;
}

// Method to compute the distance between this vector and another
double vector2d::distance(const vector2d& other) const {
  return std::sqrt((x - other.x) * (x - other.x) +
                   (y - other.y) * (y - other.y));
}

// Method to compute the squared distance between this vector and another
double vector2d::distanceSquared(const vector2d& other) const {
  return (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
}

// Method to compute the angle between this vector and another
double vector2d::angle(const vector2d& other) const {
  return std::acos(dot(other) / (magnitude() * other.magnitude()));
}

// Method to project this vector onto another
vector2d vector2d::project(const vector2d& other) const {
  return other * (dot(other) / other.dot(other));
}

// Method to reflect this vector off a surface with the given normal
vector2d vector2d::reflect(const vector2d& normal) const {
  return *this - normal * 2 * dot(normal);
}

// Method to perform linear interpolation between this vector and another
vector2d vector2d::lerp(const vector2d& other, double t) const {
  return *this * (1 - t) + other * t;
}

// Method to perform spherical linear interpolation between this vector and
// another
vector2d vector2d::slerp(const vector2d& other, double t) const {
  double dot = this->dot(other) / (magnitude() * other.magnitude());
  double theta = std::acos(dot) * t;
  vector2d relativeVec = other - *this * dot;
  relativeVec = relativeVec.normalise();
  return ((*this * std::cos(theta)) + (relativeVec * std::sin(theta)));
}

// Method to rotate this vector by a given angle (in radians)
vector2d vector2d::rotate(double angleRad) const {
  double cosA = std::cos(angleRad);
  double sinA = std::sin(angleRad);
  return vector2d(x * cosA - y * sinA, x * sinA + y * cosA);
}

// Method to rotate this vector by a given angle (in radians) around a given
// origin
vector2d vector2d::rotate(double angleRad, vector2d origin) const {
  // Translate to origin, rotate, then translate back
  return (*this - origin).rotate(angleRad) + origin;
}

// Method to compute a vector that is perpendicular to this one
vector2d vector2d::perp() const { return vector2d(-y, x); }

// Destructor
vector2d::~vector2d() {}

// Friend Functions

// Overloaded operator* for scalar multiplication with vector
vector2d operator*(double scalar, const vector2d& vec) { return vec * scalar; }

// Overloaded operator/ for scalar division with vector
vector2d operator/(double scalar, const vector2d& vec) { return vec / scalar; }

// Function to calculate and return the absolute value of the vector
vector2d abs(const vector2d& vec) {
  return vector2d(std::fabs(vec.x), std::fabs(vec.y));
}

// Function to calculate and return the minimum of two vectors
vector2d min(const vector2d& vec1, const vector2d& vec2) {
  return vector2d(std::min(vec1.x, vec2.x), std::min(vec1.y, vec2.y));
}

// Function to calculate and return the maximum of two vectors
vector2d max(const vector2d& vec1, const vector2d& vec2) {
  return vector2d(std::max(vec1.x, vec2.x), std::max(vec1.y, vec2.y));
}

// Function to calculate and return the floor of a vector
vector2d floor(const vector2d& vec) {
  return vector2d(std::floor(vec.x), std::floor(vec.y));
}

// Function to calculate and return the ceiling of a vector
vector2d ceil(const vector2d& vec) {
  return vector2d(std::ceil(vec.x), std::ceil(vec.y));
}

// Function to calculate and return the rounded value of a vector
vector2d round(const vector2d& vec) {
  return vector2d(std::round(vec.x), std::round(vec.y));
}

// Function to calculate and return the modulus of a vector and a value
vector2d fmod(const vector2d& vec, double val) {
  return vector2d(std::fmod(vec.x, val), std::fmod(vec.y, val));
}

// Function to calculate and return the minimum of a vector and a value
vector2d fmin(const vector2d& vec, double val) {
  return vector2d(std::fmin(vec.x, val), std::fmin(vec.y, val));
}

// Function to calculate and return the maximum of a vector and a value
vector2d fmax(const vector2d& vec, double val) {
  return vector2d(std::fmax(vec.x, val), std::fmax(vec.y, val));
}

// Overloaded operator* for matrix multiplication with vector
vector2d operator*(const matrix22d& mat, const vector2d& vec) {
  return vector2d(mat[0][0] * vec.x + mat[1][0] * vec.y,
                  mat[0][1] * vec.x + mat[1][1] * vec.y);
}

// Overloaded operator*= for matrix multiplication with vector
vector2d& operator*=(vector2d& vec, const matrix22d& mat) {
  vec = mat * vec;
  return vec;
}