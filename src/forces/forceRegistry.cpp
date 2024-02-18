#include "../../include/forces/forceRegistry.hpp"

forceRegistry::forceRegistry() {}

forceRegistry::~forceRegistry() {}

void forceRegistry::add(rigidBody* body, forceGenerator* fg) {
  forceRegistration fr(fg, body);
  registery.push_back(fr);
}

void forceRegistry::remove(rigidBody* body, forceGenerator* fg) {
  registery.erase(std::remove_if(registery.begin(), registery.end(),
                                 [&](const forceRegistration& reg) {
                                   return reg == forceRegistration(fg, body);
                                 }),
                  registery.end());
}

void forceRegistry::clear() { registery.clear(); }

void forceRegistry::updateForces(double dt) {
  for (auto& reg : registery) {
    reg.getForceGenerator()->updateForce(reg.getRigidBody(), dt);
  }
}

void forceRegistry::zeroForces() {
  for (auto& reg : registery) {
    // TODO: Add a clearAccumulators() method to rigidBody
    reg.getRigidBody()->clearAccumulators();
  }
}