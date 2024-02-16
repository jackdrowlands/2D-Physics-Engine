#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include "include/datatypes/vector2f.hpp"

class rigidBody {
 private:
  vector2f position;
  float rotation;

 public:
  rigidBody();
  ~rigidBody();
  vector2f getPosition();
  void setPosition(vector2f position);
  float getRotation();
  void setRotation(float rotation);
};

#endif  // RIGIDBODY_HPP
