#include "../include/utils.hpp"

// Function to compare two doubles for equality with a tolerance
// The tolerance is the maximum relative error
bool utils::compare(double a, double b) {
  return std::abs(a - b) <=
         std::numeric_limits<double>::epsilon() *
             std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

// Function to compare two vector2d for equality with a tolerance
// The tolerance is the maximum relative error
bool utils::compare(vector2d a, vector2d b) {
  return compare(a.x, b.x) && compare(a.y, b.y);
}

// Function to compare two doubles for equality with a given tolerance
// The tolerance is the maximum relative error
bool utils::compare(double a, double b, double epsilon) {
  return std::abs(a - b) <=
         epsilon * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

// Function to compare two vector2d for equality with a given tolerance
// The tolerance is the maximum relative error
bool utils::compare(vector2d a, vector2d b, double epsilon) {
  return compare(a.x, b.x, epsilon) && compare(a.y, b.y, epsilon);
}