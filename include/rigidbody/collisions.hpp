#ifndef COLLISSIONS_HPP
#define COLLISSIONS_HPP

#include <vector>

#include "../datatypes/vector2d.hpp"
#include "../primitives/circle.hpp"
#include "collisionManifold.hpp"

class collisions {
 private:
  /* data */
 public:
  static collisionManifold circleToCircle(circle& a, circle& b);
};

#endif  // COLLISSIONS_HPP