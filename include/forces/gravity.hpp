#ifndef GRAVITY_HPP
#define GRAVITY_HPP

#include "forceGenerator.hpp"

class gravity : public forceGenerator {
 private:
  vector2d gravityForce;

 public:
  gravity(vector2d gravity);
  ~gravity();
  void updateForce(rigidBody* body, double dt);
};

#endif  // GRAVITY_HPP