#ifndef AABB_HPP
#define AABB_HPP

#include "include/datatypes/vector2f.hpp"
#include "include/rigidbody/rigidBody.hpp"

// Axis Aligned Bounding Box
class AABB {
 private:
  vector2f size;
  rigidBody body;

 public:
  AABB();
  AABB(vector2f size, rigidBody body);
  AABB(vector2f min, vector2f max);
  ~AABB();
  vector2f getMin();
  vector2f getMax();
};

#endif  // AABB_HPP