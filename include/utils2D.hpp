#ifndef UTILS2D_HPP
#define UTILS2D_HPP
#include <algorithm>
#include <cmath>
#include <limits>

#include "datatypes/vector2d.hpp"
class utils2D {
 public:
  utils2D();
  ~utils2D();
  static bool compare(double a, double b);
  static bool compare(vector2d a, vector2d b);
  static bool compare(double a, double b, double epsilon);
  static bool compare(vector2d a, vector2d b, double epsilon);
};

#endif  // UTILS_HPP