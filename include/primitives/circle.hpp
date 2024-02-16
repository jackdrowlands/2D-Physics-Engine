#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "../../include/rigidbody/rigidBody.hpp"

class circle {
 private:
  double radius;
  rigidBody body;

 public:
  circle(double radius);
  ~circle();

  double getRadius();
  vector2d getCentre();
};

#endif  // CIRCLE_HPP