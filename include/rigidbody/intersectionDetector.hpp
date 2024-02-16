#ifndef INTERSECTIONDETECTOR_HPP
#define INTERSECTIONDETECTOR_HPP

#include "../../include/datatypes/line.hpp"
#include "../../include/datatypes/vector2d.hpp"
#include "../../include/primitives/AABB.hpp"
#include "../../include/primitives/box.hpp"
#include "../../include/primitives/circle.hpp"
#include "../../include/primitives/ray.hpp"
#include "../../include/primitives/raycastResult.hpp"
#include "../../include/utils.hpp"

class intersectionDetector {
 private:
  /* data */
 public:
  intersectionDetector(/* args */);
  ~intersectionDetector();

  static bool pointOnLine(vector2d point, line line);

  static bool pointInCircle(vector2d point, circle circle);

  static bool pointInAABB(vector2d point, AABB aabb);

  static bool pointInBox(vector2d point, box box);

  static bool lineCircle(line line, circle circle);

  static bool lineAABB(line line, AABB aabb);

  static bool lineBox(line line, box box);

  static bool rayCircle(ray ray, circle circle, raycastResult &result);

  static bool rayCircle(ray ray, circle circle);
};

#endif  // INTERSECTIONDETECTOR_HPP