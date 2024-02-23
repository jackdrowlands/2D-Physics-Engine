#include "../../include/primitives/AABB.hpp"

// Default constructor, initializes size to zero
AABB::AABB() : size(0, 0) {}

// Constructor that initializes AABB with given size and rigidBody
AABB::AABB(vector2d size, rigidBody body) : size(size), body(body) {}

// Constructor that initializes AABB with given min and max vectors
// The size is calculated as the difference between max and min
// The position of the body is set to the center of the AABB
AABB::AABB(vector2d min, vector2d max) : size(max - min) {
  body.setPosition(min + size / 2);
}

// Destructor
AABB::~AABB() {}

// Function to get the min vector of the AABB
// Calculated as the position of the body minus half the size
vector2d AABB::getMin() { return body.getPosition() - size / 2; }

// Function to get the max vector of the AABB
// Calculated as the position of the body plus half the size
vector2d AABB::getMax() { return body.getPosition() + size / 2; }

// Function to set the rigidBody of the AABB
void AABB::setRigidBody(rigidBody body) { this->body = body; }

// Function to set the size of the AABB
void AABB::setSize(vector2d size) { this->size = size; }