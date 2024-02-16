#include "../../include/primitives/AABB.hpp"

AABB::AABB() : size(0, 0) {}

AABB::AABB(vector2d size, rigidBody body) : size(size), body(body) {}

AABB::AABB(vector2d min, vector2d max) : size(max - min) {
  body.setPosition(min + size / 2);
}

AABB::~AABB() {}

vector2d AABB::getMin() { return body.getPosition() - size / 2; }

vector2d AABB::getMax() { return body.getPosition() + size / 2; }