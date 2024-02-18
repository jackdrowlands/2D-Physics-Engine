#ifndef PHYSICSSYSTEM_HPP
#define PHYSICSSYSTEM_HPP

#include <vector>

#include "forces/forceRegistry.hpp"
#include "forces/gravity.hpp"

class physicsSystem {
 private:
  forceRegistry registry;
  std::vector<rigidBody*> rigidBodies;
  gravity gravityForce;
  double fixedDeltaTime;

 public:
  physicsSystem(double fixedDeltaTime, vector2d gravity);
  ~physicsSystem();
  void update(double dt);
  void fixedUpdate();
  void addRigidBody(rigidBody* body);
};

#endif  // PHYSICSSYSTEM_HPP