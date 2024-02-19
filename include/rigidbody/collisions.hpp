#ifndef COLLISSIONS_HPP
#define COLLISSIONS_HPP

#include <iostream>
#include <vector>

#include "../datatypes/vector2d.hpp"
#include "../primitives/circle.hpp"
#include "collisionManifold.hpp"

class collisions {
 private:
  /* data */
 public:
  static collisionManifold circleToCircle(circle& a, circle& b);

  static collisionManifold* findCollisionFeatures(collider* c1, collider* c2);
};

#endif  // COLLISSIONS_HPP