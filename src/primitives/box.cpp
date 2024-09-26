#include "../../include/primitives/box.hpp"

// Default constructor, initializes size to zero
box::box() : size(0, 0) {}

// Constructor that initializes box with given size
box::box(vector2d size) : size(size) {}

// Constructor that initializes box with given min and max vectors
// The size is calculated as the difference between max and min
box::box(vector2d min, vector2d max) : size(max - min) {}

// Destructor
box::~box() {}

// Function to get the local min vector of the box
// Calculated as the position of the body minus half the size
vector2d box::getLocalMin() { return body->getPosition() - size / 2; }

// Function to get the local max vector of the box
// Calculated as the position of the body plus half the size
vector2d box::getLocalMax() { return body->getPosition() + size / 2; }

// Function to get the vertices of the box
std::vector<vector2d> box::getVertices() {
  vector2d min = getLocalMin();
  vector2d max = getLocalMax();

  // Define the vertices of the box
  std::vector<vector2d> vertices = {
      vector2d(min.x, min.y), vector2d(max.x, min.y), vector2d(max.x, max.y),
      vector2d(min.x, max.y)};

  // If the box is rotated, rotate the vertices accordingly
  if (body->getRotation() != 0.0f) {
    for (auto& vertex : vertices) {
      vertex = body->getPosition() +
               (vertex - body->getPosition()).rotate(body->getRotation());
    }
  }
  return vertices;
}

// Function to get the rigidBody of the box
rigidBody* box::getRigidBody() { return body; }

// Function to get the size of the box
vector2d box::getSize() { return size; }

// Function to get the half size of the box
vector2d box::getHalfSize() { return size / 2; }

// Function to set the rigidBody of the box using a pointer
void box::setRigidBody(rigidBody* body) { this->body = body; }

// Function to set the size of the box
void box::setSize(vector2d size) { this->size = size; }

// Function to get the type of the primitive
std::string box::getType() { return "box"; }