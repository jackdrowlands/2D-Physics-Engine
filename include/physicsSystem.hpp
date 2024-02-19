#ifndef PHYSICSSYSTEM_HPP
#define PHYSICSSYSTEM_HPP

#include <vector>

#include "forces/forceRegistry.hpp"
#include "forces/gravity.hpp"
#include "rigidbody/collisions.hpp"

class physicsSystem {
 private:
  forceRegistry registry;
  gravity gravityForce;

  std::vector<rigidBody*> rigidBodies;
  std::vector<rigidBody*> bodies1;
  std::vector<rigidBody*> bodies2;
  std::vector<collisionManifold> manifolds;

  double fixedDeltaTime;

 public:
  physicsSystem(double fixedDeltaTime, vector2d gravity);
  ~physicsSystem();
  void update(double dt);
  void fixedUpdate();
  void addRigidBody(rigidBody* body);
};

#endif  // PHYSICSSYSTEM_HPP