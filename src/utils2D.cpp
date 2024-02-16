#include "../include/utils2D.hpp"

utils2D::utils2D() {}

utils2D::~utils2D() {}

bool utils2D::compare(double a, double b) {
  return std::abs(a - b) <=
         std::numeric_limits<double>::epsilon() *
             std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

bool utils2D::compare(vector2d a, vector2d b) {
  return compare(a.x, b.x) && compare(a.y, b.y);
}

bool utils2D::compare(double a, double b, double epsilon) {
  return std::abs(a - b) <=
         epsilon * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

bool utils2D::compare(vector2d a, vector2d b, double epsilon) {
  return compare(a.x, b.x, epsilon) && compare(a.y, b.y, epsilon);
}
