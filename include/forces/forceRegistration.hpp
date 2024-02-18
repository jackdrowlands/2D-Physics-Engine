#ifndef FORCEREGRISTRATION_HPP
#define FORCEREGRISTRATION_HPP

#include "forceGenerator.hpp"

class forceRegistration {
 private:
  forceGenerator* fg;
  rigidBody* body;

 public:
  forceRegistration(forceGenerator* fg, rigidBody* body);
  ~forceRegistration();

  // friend class forceRegistry;
  forceGenerator* getForceGenerator();
  rigidBody* getRigidBody();

  // equality operator
  bool operator==(const forceRegistration& other) const;
};

#endif  // FORCEREGRISTRATION_HPP