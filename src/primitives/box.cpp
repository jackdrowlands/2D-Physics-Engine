#include "../../include/primitives/box.hpp"

box::box() : size(0, 0) {}

box::box(vector2f size) : size(size) {}

box::box(vector2f min, vector2f max) : size(max - min) {}

box::~box() {}

vector2f box::getMin() { return body.getPosition() - size / 2; }

vector2f box::getMax() { return body.getPosition() + size / 2; }

std::vector<vector2f> box::getVertices() {
  vector2f min = getMin();
  vector2f max = getMax();

  std::vector<vector2f> vertices = {
      vector2f(min.x, min.y), vector2f(max.x, min.y), vector2f(max.x, max.y),
      vector2f(min.x, max.y)};

  if (body.getRotation() != 0.0f) {
    for (auto& vertex : vertices) {
      vertex = body.getPosition() +
               (vertex - body.getPosition()).rotate(body.getRotation());
    }
  }
  return vertices;
}