#include "../../include/rigidbody/rigidBody.hpp"

// Default constructor
rigidBody::rigidBody()
    : rotation(0.0f),
      angularDamping(0.0f),
      linearDamping(0.0f),
      fixedRotation(false) {}

// Constructor with position and rotation
rigidBody::rigidBody(vector2d position, double rotation) : position(position) {
  this->rotation = rotation;
}

// Constructor with position, rotation, and mass
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

// Constructor with all properties
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

// Destructor
rigidBody::~rigidBody() {}

// Get position
vector2d rigidBody::getPosition() { return position; }

// Set position
void rigidBody::setPosition(vector2d position) { this->position = position; }

// Get rotation
double rigidBody::getRotation() { return rotation; }

// Set rotation
void rigidBody::setRotation(double rotation) { this->rotation = rotation; }

// Set position and rotation
void rigidBody::setTransform(vector2d position, double rotation) {
  this->position = position;
  this->rotation = rotation;
}

// Set position
void rigidBody::setTransform(vector2d position) { this->position = position; }

// Get mass
double rigidBody::getMass() { return mass; }

// Set mass and calculate inverse mass
void rigidBody::setMass(double mass) {
  this->mass = mass;
  if (mass == 0) {
    inverseMass = 0;
  } else {
    inverseMass = 1 / mass;
  }
}

// Get inverse mass
double rigidBody::getInverseMass() { return inverseMass; }

// Update physics properties
void rigidBody::physicsUpdate(double dt) {
  if (inverseMass == 0) {
    return;
  }
  vector2d acceleration = forceAccum * inverseMass;
  linearVelocity += acceleration * dt;
  position += linearVelocity * dt;
  rotation += angularVelocity * dt;

  syncCollisionTransforms();
  clearAccumulators();
}

// Clear force accumulators
void rigidBody::clearAccumulators() { forceAccum = vector2d(0, 0); }

// Sync collision transforms
void rigidBody::syncCollisionTransforms() {}

// Add force to the rigid body
void rigidBody::addForce(vector2d force) { forceAccum += force; }

// Check if the rigid body has infinite mass
bool rigidBody::hasInfiniteMass() { return mass == 0; }

// Set collider
void rigidBody::setCollider(collider* collider) { this->col = collider; }

// Get collider
collider* rigidBody::getCollider() { return col; }

// Set linear velocity
void rigidBody::setLinearVelocity(vector2d linearVelocity) {
  this->linearVelocity = linearVelocity;
}

// Get linear velocity
vector2d rigidBody::getLinearVelocity() { return linearVelocity; }

// Get coefficient of restitution
double rigidBody::getCor() { return cor; }

// Set coefficient of restitution
void rigidBody::setCor(double cor) { this->cor = cor; }

// Set angular velocity
void rigidBody::setAngularVelocity(double angularVelocity) {
  this->angularVelocity = angularVelocity;
}

// Get angular velocity
double rigidBody::getAngularVelocity() { return angularVelocity; }

// Set centre of mass
void rigidBody::setCentreOfMass(vector2d centreOfMass) {
  this->centreOfMass = centreOfMass;
}

// Get centre of mass
vector2d rigidBody::getCentreOfMass() { return position; }