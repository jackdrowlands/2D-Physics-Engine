#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

#include <algorithm>
#include <cmath>
#include <iostream>

class vector2d {
 public:
  double x;
  double y;

  // Constructor
  vector2d();
  vector2d(double x, double y);
  vector2d(double x);

  // Operator Overloads
  vector2d operator+(const vector2d& other) const;
  vector2d operator-(const vector2d& other) const;
  vector2d operator*(const double& scalar) const;
  vector2d operator*(const vector2d& other) const;
  vector2d operator/(const double& scalar) const;
  vector2d& operator+=(const vector2d& other);
  vector2d& operator-=(const vector2d& other);
  vector2d& operator*=(const double& scalar);
  vector2d& operator*=(const vector2d& other);
  vector2d& operator/=(const double& scalar);
  bool operator==(const vector2d& other) const;
  bool operator!=(const vector2d& other) const;
  double& operator[](int index);
  const double& operator[](int index) const;
  vector2d operator-() const;

  // Methods
  double magnitude() const;
  vector2d normalise() const;
  double dot(const vector2d& other) const;
  double cross(const vector2d& other) const;
  double distance(const vector2d& other) const;
  double distanceSquared(const vector2d& other) const;
  double angle(const vector2d& other) const;
  vector2d project(const vector2d& other) const;
  vector2d reflect(const vector2d& normal) const;
  vector2d lerp(const vector2d& other, double t) const;
  vector2d slerp(const vector2d& other, double t) const;
  vector2d rotate(double angleRad) const;
  vector2d rotate(double angleRad, vector2d origin) const;
  vector2d perp() const;

  // Destructor
  ~vector2d();

  // Friend Functions
  friend vector2d operator*(double scalar, const vector2d& vec);
  friend vector2d operator/(double scalar, const vector2d& vec);
  friend vector2d operator-(const vector2d& vec);
  friend vector2d abs(const vector2d& vec);
  friend vector2d min(const vector2d& vec1, const vector2d& vec2);
  friend vector2d max(const vector2d& vec1, const vector2d& vec2);
  friend vector2d floor(const vector2d& vec);
  friend vector2d ceil(const vector2d& vec);
  friend vector2d round(const vector2d& vec);
  friend vector2d fmod(const vector2d& vec, double val);
  friend vector2d fmin(const vector2d& vec, double val);
  friend vector2d fmax(const vector2d& vec, double val);
};

#endif  // VECTOR2D_HPP
