#ifndef UTILS_HPP
#define UTILS_HPP
#include <algorithm>
#include <cmath>
#include <limits>

#include "datatypes/vector2d.hpp"
class utils {
 public:
  utils();
  ~utils();
  static bool compare(double a, double b);
  static bool compare(vector2d a, vector2d b);
  static bool compare(double a, double b, double epsilon);
  static bool compare(vector2d a, vector2d b, double epsilon);
};

#endif  // UTILS_HPP