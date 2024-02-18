#include "../../include/rigidbody/rigidBody.hpp"

rigidBody::rigidBody()
    : rotation(0.0f),
      angularDamping(0.0f),
      linearDamping(0.0f),
      fixedRotation(false) {}

rigidBody::rigidBody(vector2d position, double rotation) : position(position) {
  this->rotation = rotation;
}

rigidBody::rigidBody(vector2d position, double rotation, double mass)
    : position(position),
      rotation(rotation),
      linearVelocity(0),
      angularVelocity(0),
      linearDamping(0),
      angularDamping(0),
      fixedRotation(false) {
  this->setMass(mass);
};

rigidBody::rigidBody(vector2d position, double rotation,
                     vector2d linearVelocity, double angularVelocity,
                     double linearDamping, double angularDamping,
                     bool fixedRotation)
    : position(position),
      rotation(rotation),
      linearVelocity(linearVelocity),
      angularVelocity(angularVelocity),
      linearDamping(linearDamping),
      angularDamping(angularDamping),
      fixedRotation(fixedRotation) {}

rigidBody::~rigidBody() {}

vector2d rigidBody::getPosition() { return position; }

void rigidBody::setPosition(vector2d position) { this->position = position; }

double rigidBody::getRotation() { return rotation; }

void rigidBody::setRotation(double rotation) { this->rotation = rotation; }

void rigidBody::setTransform(vector2d position, double rotation) {
  this->position = position;
  this->rotation = rotation;
}

void rigidBody::setTransform(vector2d position) { this->position = position; }

double rigidBody::getMass() { return mass; }

void rigidBody::setMass(double mass) {
  this->mass = mass;
  if (mass == 0) {
    inverseMass = 0;
  } else {
    inverseMass = 1 / mass;
  }
}

double rigidBody::getInverseMass() { return inverseMass; }

void rigidBody::physicsUpdate(double dt) {
  if (inverseMass == 0) {
    return;
  }
  vector2d acceleration = forceAccum * inverseMass;
  linearVelocity += acceleration * dt;
  position += linearVelocity * dt;

  syncCollisionTransforms();
  clearAccumulators();
}

void rigidBody::clearAccumulators() { forceAccum = vector2d(0, 0); }

void rigidBody::syncCollisionTransforms() {}

void rigidBody::addForce(vector2d force) { forceAccum += force; }