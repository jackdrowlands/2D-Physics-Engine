#ifndef AABB_HPP
#define AABB_HPP

#include "../../include/datatypes/vector2d.hpp"
#include "../../include/rigidbody/rigidBody.hpp"

// Axis Aligned Bounding Box
class AABB {
 private:
  vector2d size;
  rigidBody body;

 public:
  AABB();
  AABB(vector2d size, rigidBody body);
  AABB(vector2d min, vector2d max);
  ~AABB();
  vector2d getMin();
  vector2d getMax();
  void setRigidBody(rigidBody body);
  void setSize(vector2d size);
};

#endif  // AABB_HPP