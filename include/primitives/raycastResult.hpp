#ifndef RAAYCASTRESULT_HPP
#define RAAYCASTRESULT_HPP

#include "../../include/datatypes/vector2d.hpp"

class raycastResult {
 private:
  vector2d point;
  vector2d normal;
  double t;
  bool hit;

 public:
  raycastResult();
  ~raycastResult();
  void init(vector2d point, vector2d direction, double t, bool hit);
  static void reset(raycastResult& result);
  vector2d getPoint();
  vector2d getNormal();
  double getT();
};

#endif  // RAAYCASTRESULT_HPP