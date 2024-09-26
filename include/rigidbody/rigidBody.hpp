#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include "../../include/datatypes/vector2d.hpp"
#include "../include/primitives/collider.hpp"

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
  collider* col;
  vector2d centreOfMass;

  double cor = 1.0f;
  std::vector<int> quadtreeIndices;

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
  bool hasInfiniteMass();
  void setCollider(collider* collider);
  collider* getCollider();
  void setLinearVelocity(vector2d linearVelocity);
  vector2d getLinearVelocity();
  double getCor();
  void setCor(double cor);
  void setAngularVelocity(double angularVelocity);
  double getAngularVelocity();
  void setCentreOfMass(vector2d centreOfMass);
  vector2d getCentreOfMass();
  void setQuadtreeIndex(std::vector<int> indices);
  std::vector<int> getQuadtreeIndices();
};

#endif  // RIGIDBODY_HPP
