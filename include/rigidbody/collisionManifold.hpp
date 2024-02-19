#ifndef COLLISIONMANIFOLD_HPP
#define COLLISIONMANIFOLD_HPP

#include <vector>

#include "../datatypes/vector2d.hpp"

class collisionManifold {
 private:
  vector2d normal;
  std::vector<vector2d> contactPoint;
  double depth;
  bool isColliding;

 public:
  collisionManifold(vector2d normal, double depth, bool isColliding);
  ~collisionManifold();
  vector2d getNormal();
  std::vector<vector2d> getContactPoint();
  double getDepth();
  bool getIsColliding();
  void addContactPoint(vector2d point);
};

#endif  // COLLISIONMANIFOLD_HPP