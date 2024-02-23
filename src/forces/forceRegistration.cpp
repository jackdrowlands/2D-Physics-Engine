#include "../../include/forces/forceRegistration.hpp"

// Constructor that initializes forceRegistration with given forceGenerator and
// rigidBody
forceRegistration::forceRegistration(forceGenerator* fg, rigidBody* body)
    : fg(fg), body(body) {}

// Destructor
forceRegistration::~forceRegistration() {}

// Function to get the forceGenerator
forceGenerator* forceRegistration::getForceGenerator() { return fg; }

// Function to get the rigidBody
rigidBody* forceRegistration::getRigidBody() { return body; }

// Overloaded operator== to compare two forceRegistrations
bool forceRegistration::operator==(const forceRegistration& other) const {
  return (fg == other.fg) && (body == other.body);
}