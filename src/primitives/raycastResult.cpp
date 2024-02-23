#include "../../include/primitives/raycastResult.hpp"

// Default constructor, initializes t to -1 and hit to false
raycastResult::raycastResult() : t(-1), hit(false) {}

// Destructor
raycastResult::~raycastResult() {}

// Function to initialize a raycastResult with given point, direction, t, and
// hit The point is the point of intersection The direction is the direction of
// the ray t is the parameter of the ray equation hit is a boolean indicating
// whether the ray hit an object
void raycastResult::init(vector2d point, vector2d direction, double t,
                         bool hit) {
  this->point = point;
  this->normal = direction;
  this->t = t;
  this->hit = hit;
}

// Function to reset a raycastResult to its default state
void raycastResult::reset(raycastResult& result) {
  result.point = vector2d();
  result.normal = vector2d();
  result.t = -1;
  result.hit = false;
}

// Function to get the point of intersection
vector2d raycastResult::getPoint() { return point; }

// Function to get the direction of the ray
vector2d raycastResult::getNormal() { return normal; }

// Function to get the parameter of the ray equation
double raycastResult::getT() { return t; }