#ifndef RAY_HPP
#define RAY_HPP

#include "../../include/datatypes/vector2d.hpp"

class ray {
 private:
  vector2d origin;
  vector2d direction;

 public:
  ray(vector2d origin, vector2d direction);
  ~ray();
  vector2d getOrigin();
  vector2d getDirection();
};

#endif  // RAY_HPP