#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "../datatypes/vector2d.hpp"
#include "../render/renderer.hpp"

class collider {
 private:
  /* data */
 protected:
  vector2d offset;
  rigidBody body;

 public:
  collider(/* args */);
  ~collider();
  virtual int getType() = 0;
  virtual void setRigidBody(rigidBody* body);
  virtual rigidBody& getRigidBody();
};

#endif  // COLLIDER_HPP