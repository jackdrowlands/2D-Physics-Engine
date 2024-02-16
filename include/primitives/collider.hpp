#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "include/datatypes/vector2f.hpp"

class collider {
 private:
  /* data */
 protected:
  vector2f offset;

 public:
  collider(/* args */);
  ~collider();
  // TODO: Add a pure virtual function to get the inertia tensor
  virtual void getInertiaTensor(float mass) = 0;
};

#endif  // COLLIDER_HPP