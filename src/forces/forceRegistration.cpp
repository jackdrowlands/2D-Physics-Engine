#include "../../include/forces/forceRegistration.hpp"

forceRegistration::forceRegistration(forceGenerator* fg, rigidBody* body)
    : fg(fg), body(body) {}

forceRegistration::~forceRegistration() {}

forceGenerator* forceRegistration::getForceGenerator() { return fg; }

rigidBody* forceRegistration::getRigidBody() { return body; }

bool forceRegistration::operator==(const forceRegistration& other) const {
  return (fg == other.fg) && (body == other.body);
}