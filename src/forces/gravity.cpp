#include "../../include/forces/gravity.hpp"

// Constructor that initializes gravity with given gravity vector
gravity::gravity(vector2d gravity) : gravityForce(gravity) {}

// Destructor
gravity::~gravity() {}

// Function to update the force of a rigidBody
void gravity::updateForce(rigidBody* body, double dt) {
  // If the body's mass is zero, we don't apply any force
  if (body->getMass() == 0) {
    return;
  }

  // Apply the force of gravity (mass * acceleration due to gravity)
  body->addForce(gravityForce * body->getMass());
}