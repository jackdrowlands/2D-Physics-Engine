#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "../../include/rigidbody/rigidBody.hpp"
#include "collider.hpp"

class circle : public collider {
 private:
  double radius;
  rigidBody body;

 public:
  circle(double radius);
  circle(double radius, vector2d centre);
  ~circle();

  double getRadius();
  vector2d getCentre();
  void setCentre(vector2d newCentre);
  void setRadius(double newRadius);
  rigidBody& getRigidBody();
  void setRigidBody(rigidBody body);
  std::string getType();
};

#endif  // CIRCLE_HPP