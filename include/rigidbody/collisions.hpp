#ifndef COLLISSIONS_HPP
#define COLLISSIONS_HPP

#include <iostream>
#include <vector>

#include "../datatypes/matrix22d.hpp"
#include "../datatypes/vector2d.hpp"
#include "../primitives/box.hpp"
#include "../primitives/circle.hpp"
#include "../rigidbody/rigidBody.hpp"
#include "collisionManifold.hpp"
#include "intersectionDetector.hpp"

class collisions {
 private:
  /* data */
 public:
  static collisionManifold circleToCircle(circle& a, circle& b);

  static collisionManifold circleToBox(circle& a, box& b);

  static collisionManifold boxToBox(box& a, box& b);

  static collisionManifold* findCollisionFeatures(collider* c1, collider* c2);

  static std::vector<std::vector<rigidBody*>> createQuadtree(
      std::vector<rigidBody*>& rigidBodies, const int amountOfQuadSQRT = 50);

  static void updateQuadtree(std::vector<std::vector<rigidBody*>>& quadtree,
                             std::vector<rigidBody*>& rigidBodies,
                             const int amountOfQuadSQRT = 10);
};

#endif  // COLLISSIONS_HPP