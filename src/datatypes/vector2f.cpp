#include "../../include/datatypes/vector2f.hpp"

// Constructor
vector2f::vector2f(float x, float y) : x(x), y(y) {}

// Operator Overloads
vector2f vector2f::operator+(const vector2f& other) const {
  return vector2f(x + other.x, y + other.y);
}

vector2f vector2f::operator-(const vector2f& other) const {
  return vector2f(x - other.x, y - other.y);
}

vector2f vector2f::operator*(const float& scalar) const {
  return vector2f(x * scalar, y * scalar);
}

vector2f vector2f::operator/(const float& scalar) const {
  return vector2f(x / scalar, y / scalar);
}

vector2f& vector2f::operator+=(const vector2f& other) {
  x += other.x;
  y += other.y;
  return *this;
}

vector2f& vector2f::operator-=(const vector2f& other) {
  x -= other.x;
  y -= other.y;
  return *this;
}

vector2f& vector2f::operator*=(const float& scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}

vector2f& vector2f::operator/=(const float& scalar) {
  x /= scalar;
  y /= scalar;
  return *this;
}

bool vector2f::operator==(const vector2f& other) const {
  return x == other.x && y == other.y;
}

bool vector2f::operator!=(const vector2f& other) const {
  return x != other.x || y != other.y;
}

float& vector2f::operator[](int index) { return index == 0 ? x : y; }

const float& vector2f::operator[](int index) const {
  return index == 0 ? x : y;
}

vector2f vector2f::operator-() const { return vector2f(-x, -y); }

// Methods
float vector2f::magnitude() const { return std::sqrt(x * x + y * y); }

vector2f vector2f::normalise() const {
  float mag = magnitude();
  return mag == 0 ? vector2f(0, 0) : vector2f(x / mag, y / mag);
}

float vector2f::dot(const vector2f& other) const {
  return x * other.x + y * other.y;
}

float vector2f::cross(const vector2f& other) const {
  return x * other.y - y * other.x;
}

float vector2f::distance(const vector2f& other) const {
  return std::sqrt((x - other.x) * (x - other.x) +
                   (y - other.y) * (y - other.y));
}

float vector2f::angle(const vector2f& other) const {
  return std::acos(dot(other) / (magnitude() * other.magnitude()));
}

vector2f vector2f::project(const vector2f& other) const {
  return other * (dot(other) / other.dot(other));
}

vector2f vector2f::reflect(const vector2f& normal) const {
  return *this - normal * 2 * dot(normal);
}

vector2f vector2f::lerp(const vector2f& other, float t) const {
  return *this * (1 - t) + other * t;
}

vector2f vector2f::slerp(const vector2f& other, float t) const {
  float dot = this->dot(other) / (magnitude() * other.magnitude());
  float theta = std::acos(dot) * t;
  vector2f relativeVec = other - *this * dot;
  relativeVec = relativeVec.normalise();
  return ((*this * std::cos(theta)) + (relativeVec * std::sin(theta)));
}

vector2f vector2f::rotate(float angle) const {
  float rad = angle;
  return vector2f(x * std::cos(rad) - y * std::sin(rad),
                  x * std::sin(rad) + y * std::cos(rad));
}

vector2f vector2f::perp() const { return vector2f(-y, x); }

// Destructor
vector2f::~vector2f() {}

// Friend Functions

vector2f operator*(float scalar, const vector2f& vec) { return vec * scalar; }

vector2f operator/(float scalar, const vector2f& vec) { return vec / scalar; }

vector2f abs(const vector2f& vec) {
  return vector2f(std::fabs(vec.x), std::fabs(vec.y));
}

vector2f min(const vector2f& vec1, const vector2f& vec2) {
  return vector2f(std::min(vec1.x, vec2.x), std::min(vec1.y, vec2.y));
}

vector2f max(const vector2f& vec1, const vector2f& vec2) {
  return vector2f(std::max(vec1.x, vec2.x), std::max(vec1.y, vec2.y));
}

vector2f floor(const vector2f& vec) {
  return vector2f(std::floor(vec.x), std::floor(vec.y));
}

vector2f ceil(const vector2f& vec) {
  return vector2f(std::ceil(vec.x), std::ceil(vec.y));
}

vector2f round(const vector2f& vec) {
  return vector2f(std::round(vec.x), std::round(vec.y));
}

vector2f fmod(const vector2f& vec, float val) {
  return vector2f(std::fmod(vec.x, val), std::fmod(vec.y, val));
}

vector2f fmin(const vector2f& vec, float val) {
  return vector2f(std::fmin(vec.x, val), std::fmin(vec.y, val));
}

vector2f fmax(const vector2f& vec, float val) {
  return vector2f(std::fmax(vec.x, val), std::fmax(vec.y, val));
}
