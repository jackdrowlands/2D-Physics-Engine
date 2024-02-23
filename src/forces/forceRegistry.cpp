#include "../../include/forces/forceRegistry.hpp"

// Default constructor
forceRegistry::forceRegistry() {}

// Destructor
forceRegistry::~forceRegistry() {}

// Function to add a forceGenerator and a rigidBody to the registry
void forceRegistry::add(rigidBody* body, forceGenerator* fg) {
  forceRegistration fr(fg, body);
  registery.push_back(fr);
}

// Function to remove a forceGenerator and a rigidBody from the registry
void forceRegistry::remove(rigidBody* body, forceGenerator* fg) {
  registery.erase(std::remove_if(registery.begin(), registery.end(),
                                 [&](const forceRegistration& reg) {
                                   return reg == forceRegistration(fg, body);
                                 }),
                  registery.end());
}

// Function to clear the registry
void forceRegistry::clear() { registery.clear(); }

// Function to update forces for all registered forceGenerators and rigidBodies
void forceRegistry::updateForces(double dt) {
  for (auto& reg : registery) {
    reg.getForceGenerator()->updateForce(reg.getRigidBody(), dt);
  }
}

// Function to clear all forces from all registered rigidBodies
void forceRegistry::zeroForces() {
  for (auto& reg : registery) {
    reg.getRigidBody()->clearAccumulators();
  }
}