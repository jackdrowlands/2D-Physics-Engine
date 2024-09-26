#ifndef PHYSICSSYSTEM_HPP
#define PHYSICSSYSTEM_HPP

#include <mutex>
#include <thread>
#include <vector>

#include "forces/forceRegistry.hpp"
#include "forces/gravity.hpp"
#include "rigidbody/collisions.hpp"

class physicsSystem {
 private:
  forceRegistry registry;
  gravity gravityForce;

  std::vector<collider*> colliders;
  std::vector<std::vector<collider*>> quadtree;
  std::vector<rigidBody*> bodies1;
  std::vector<rigidBody*> bodies2;
  std::vector<collisionManifold> manifolds;

  double fixedDeltaTime;
  int impulseIterations = 6;
  std::mutex manifoldMutex;

 public:
  physicsSystem(double fixedDeltaTime, vector2d gravity);
  ~physicsSystem();
  void update(double dt);
  void fixedUpdate();
  void addCollider(collider* collider, bool addGravity);
  void applyImpulse(rigidBody& a, rigidBody& b, collisionManifold& m);
};

#endif  // PHYSICSSYSTEM_HPP