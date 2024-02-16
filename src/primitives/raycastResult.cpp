#include "../../include/primitives/raycastResult.hpp"

raycastResult::raycastResult() : t(-1), hit(false) {}

raycastResult::~raycastResult() {}

void raycastResult::init(vector2d point, vector2d direction, double t,
                         bool hit) {
  this->point = point;
  this->normal = direction;
  this->t = t;
  this->hit = hit;
}

void raycastResult::reset(raycastResult& result) {
  result.point = vector2d();
  result.normal = vector2d();
  result.t = -1;
  result.hit = false;
}

vector2d raycastResult::getPoint() { return point; }

vector2d raycastResult::getNormal() { return normal; }

double raycastResult::getT() { return t; }
