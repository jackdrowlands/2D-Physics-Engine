#include "../../include/datatypes/vector2d.hpp"

// Constructor
vector2d::vector2d() : x(0), y(0) {}
vector2d::vector2d(double x, double y) : x(x), y(y) {}

// Operator Overloads
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

bool vector2d::operator==(const vector2d& other) const {
  return x == other.x && y == other.y;
}

bool vector2d::operator!=(const vector2d& other) const {
  return x != other.x || y != other.y;
}

double& vector2d::operator[](int index) { return index == 0 ? x : y; }

const double& vector2d::operator[](int index) const {
  return index == 0 ? x : y;
}

vector2d vector2d::operator-() const { return vector2d(-x, -y); }

// Methods
double vector2d::magnitude() const { return std::sqrt(x * x + y * y); }

vector2d vector2d::normalise() const {
  double mag = magnitude();
  return mag == 0 ? vector2d(0, 0) : vector2d(x / mag, y / mag);
}

double vector2d::dot(const vector2d& other) const {
  return x * other.x + y * other.y;
}

double vector2d::cross(const vector2d& other) const {
  return x * other.y - y * other.x;
}

double vector2d::distance(const vector2d& other) const {
  return std::sqrt((x - other.x) * (x - other.x) +
                   (y - other.y) * (y - other.y));
}

double vector2d::distanceSquared(const vector2d& other) const {
  return (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
}

double vector2d::angle(const vector2d& other) const {
  return std::acos(dot(other) / (magnitude() * other.magnitude()));
}

vector2d vector2d::project(const vector2d& other) const {
  return other * (dot(other) / other.dot(other));
}

vector2d vector2d::reflect(const vector2d& normal) const {
  return *this - normal * 2 * dot(normal);
}

vector2d vector2d::lerp(const vector2d& other, double t) const {
  return *this * (1 - t) + other * t;
}

vector2d vector2d::slerp(const vector2d& other, double t) const {
  double dot = this->dot(other) / (magnitude() * other.magnitude());
  double theta = std::acos(dot) * t;
  vector2d relativeVec = other - *this * dot;
  relativeVec = relativeVec.normalise();
  return ((*this * std::cos(theta)) + (relativeVec * std::sin(theta)));
}

vector2d vector2d::rotate(double angleRad) const {
  double cosA = std::cos(angleRad);
  double sinA = std::sin(angleRad);
  return vector2d(x * cosA - y * sinA, x * sinA + y * cosA);
}

vector2d vector2d::rotate(double angleRad, vector2d origin) const {
  // Translate to origin, rotate, then translate back
  return (*this - origin).rotate(angleRad) + origin;
}

vector2d vector2d::perp() const { return vector2d(-y, x); }

// Destructor
vector2d::~vector2d() {}

// Friend Functions

vector2d operator*(double scalar, const vector2d& vec) { return vec * scalar; }

vector2d operator/(double scalar, const vector2d& vec) { return vec / scalar; }

vector2d abs(const vector2d& vec) {
  return vector2d(std::fabs(vec.x), std::fabs(vec.y));
}

vector2d min(const vector2d& vec1, const vector2d& vec2) {
  return vector2d(std::min(vec1.x, vec2.x), std::min(vec1.y, vec2.y));
}

vector2d max(const vector2d& vec1, const vector2d& vec2) {
  return vector2d(std::max(vec1.x, vec2.x), std::max(vec1.y, vec2.y));
}

vector2d floor(const vector2d& vec) {
  return vector2d(std::floor(vec.x), std::floor(vec.y));
}

vector2d ceil(const vector2d& vec) {
  return vector2d(std::ceil(vec.x), std::ceil(vec.y));
}

vector2d round(const vector2d& vec) {
  return vector2d(std::round(vec.x), std::round(vec.y));
}

vector2d fmod(const vector2d& vec, double val) {
  return vector2d(std::fmod(vec.x, val), std::fmod(vec.y, val));
}

vector2d fmin(const vector2d& vec, double val) {
  return vector2d(std::fmin(vec.x, val), std::fmin(vec.y, val));
}

vector2d fmax(const vector2d& vec, double val) {
  return vector2d(std::fmax(vec.x, val), std::fmax(vec.y, val));
}
