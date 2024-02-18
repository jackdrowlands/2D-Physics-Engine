#ifndef FORCEGENERATOR_HPP
#define FORCEGENERATOR_HPP

#include "../rigidbody/rigidBody.hpp"

class forceGenerator {
 public:
  forceGenerator();
  virtual ~forceGenerator();
  virtual void updateForce(rigidBody* body, double dt) = 0;
  
};

#endif  // FORCEGENERATOR_HPP