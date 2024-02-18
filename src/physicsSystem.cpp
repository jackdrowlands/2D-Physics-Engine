#include "../include/physicsSystem.hpp"

physicsSystem::physicsSystem(double fixedDeltaTime, vector2d gravity)
    : gravityForce(gravity), fixedDeltaTime(fixedDeltaTime) {}

physicsSystem::~physicsSystem() {
  for (auto body : rigidBodies) {
    delete body;
  }
}

void physicsSystem::update(double dt) {
  // TODO:
}

void physicsSystem::fixedUpdate() {
  registry.updateForces(fixedDeltaTime);

  for (auto& body : rigidBodies) {
    body->physicsUpdate(fixedDeltaTime);
  }
}

void physicsSystem::addRigidBody(rigidBody* body) {
  rigidBodies.push_back(body);
  registry.add(body, &gravityForce);
}