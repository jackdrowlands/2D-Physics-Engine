#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "../datatypes/vector2d.hpp"

class collider {
 private:
  /* data */
 protected:
  vector2d offset;

 public:
  collider(/* args */);
  ~collider();
  // TODO: Add a pure virtual function to get the inertia tensor
  virtual void getInertiaTensor(double mass) = 0;
};

#endif  // COLLIDER_HPP