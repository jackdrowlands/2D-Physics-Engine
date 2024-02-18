#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include "../../include/datatypes/vector2d.hpp"

class rigidBody {
 private:
  vector2d position;
  double rotation;

  vector2d linearVelocity;
  double angularVelocity;
  double linearDamping;
  double angularDamping;
  bool fixedRotation;
  double mass;
  double inverseMass;
  vector2d forceAccum;

 public:
  rigidBody();
  rigidBody(vector2d position, double rotation);
  rigidBody(vector2d position, double rotation, double mass);
  rigidBody(vector2d position, double rotation, vector2d linearVelocity,
            double angularVelocity, double linearDamping, double angularDamping,
            bool fixedRotation);
  ~rigidBody();
  vector2d getPosition();
  void setPosition(vector2d position);
  double getRotation();
  void setRotation(double rotation);
  void setTransform(vector2d position, double rotation);
  void setTransform(vector2d position);
  double getMass();
  void setMass(double mass);
  double getInverseMass();
  void physicsUpdate(double dt);
  void clearAccumulators();
  void syncCollisionTransforms();
  void addForce(vector2d force);
};

#endif  // RIGIDBODY_HPP
