#ifndef FORCEREGISTRY_HPP
#define FORCEREGISTRY_HPP

#include <vector>

#include "forceRegistration.hpp"

class forceRegistry {
 private:
  std::vector<forceRegistration> registery;

 public:
  forceRegistry();
  ~forceRegistry();
  void add(rigidBody* body, forceGenerator* fg);
  void remove(rigidBody* body, forceGenerator* fg);
  void clear();
  void updateForces(double dt);
  void zeroForces();
};

#endif  // FORCEREGISTRY_HPP