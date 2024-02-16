#include "include/primitives/AABB.hpp"

AABB::AABB() : size(0, 0) {}

AABB::AABB(vector2f centre, vector2f size) : size(size) {}

AABB::AABB(vector2f min, vector2f max) : size(max - min) {}

AABB::~AABB() {}

vector2f AABB::getMin() { return body.getPosition() - size / 2; }

vector2f AABB::getMax() { return body.getPosition() + size / 2; }