#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include "include/datatypes/vector2d.hpp"

class rigidBody {
 private:
  vector2d position;
  double rotation;

 public:
  rigidBody();
  ~rigidBody();
  vector2d getPosition();
  void setPosition(vector2d position);
  double getRotation();
  void setRotation(double rotation);
};

#endif  // RIGIDBODY_HPP
