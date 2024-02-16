#include "../../include/primitives/box.hpp"

box::box() : size(0, 0) {}

box::box(vector2d size) : size(size) {}

box::box(vector2d min, vector2d max) : size(max - min) {}

box::~box() {}

vector2d box::getMin() { return body.getPosition() - size / 2; }

vector2d box::getMax() { return body.getPosition() + size / 2; }

std::vector<vector2d> box::getVertices() {
  vector2d min = getMin();
  vector2d max = getMax();

  std::vector<vector2d> vertices = {
      vector2d(min.x, min.y), vector2d(max.x, min.y), vector2d(max.x, max.y),
      vector2d(min.x, max.y)};

  if (body.getRotation() != 0.0f) {
    for (auto& vertex : vertices) {
      vertex = body.getPosition() +
               (vertex - body.getPosition()).rotate(body.getRotation());
    }
  }
  return vertices;
}

rigidBody box::getRigidBody() { return body; }