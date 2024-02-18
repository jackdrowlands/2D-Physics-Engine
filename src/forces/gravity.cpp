#include "../../include/forces/gravity.hpp"

gravity::gravity(vector2d gravity) : gravityForce(gravity) {}

gravity::~gravity() {}

void gravity::updateForce(rigidBody* body, double dt) {
  if (body->getMass() == 0) {
    return;
  }
  body->addForce(gravityForce * body->getMass());
}