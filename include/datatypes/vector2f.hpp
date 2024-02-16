#ifndef VECTOR2F_HPP
#define VECTOR2F_HPP

#include <algorithm>
#include <cmath>
#include <iostream>

class vector2f {
 public:
  float x;
  float y;

  // Constructor
  vector2f(float x = 0.0f, float y = 0.0f);

  // Operator Overloads
  vector2f operator+(const vector2f& other) const;
  vector2f operator-(const vector2f& other) const;
  vector2f operator*(const float& scalar) const;
  vector2f operator/(const float& scalar) const;
  vector2f& operator+=(const vector2f& other);
  vector2f& operator-=(const vector2f& other);
  vector2f& operator*=(const float& scalar);
  vector2f& operator/=(const float& scalar);
  bool operator==(const vector2f& other) const;
  bool operator!=(const vector2f& other) const;
  float& operator[](int index);
  const float& operator[](int index) const;
  vector2f operator-() const;

  // Methods
  float magnitude() const;
  vector2f normalise() const;
  float dot(const vector2f& other) const;
  float cross(const vector2f& other) const;
  float distance(const vector2f& other) const;
  float angle(const vector2f& other) const;
  vector2f project(const vector2f& other) const;
  vector2f reflect(const vector2f& normal) const;
  vector2f lerp(const vector2f& other, float t) const;
  vector2f slerp(const vector2f& other, float t) const;
  vector2f rotate(float angle) const;
  vector2f perp() const;

  // Destructor
  ~vector2f();

  // Friend Functions
  friend vector2f operator*(float scalar, const vector2f& vec);
  friend vector2f operator/(float scalar, const vector2f& vec);
  friend vector2f operator-(const vector2f& vec);
  friend vector2f abs(const vector2f& vec);
  friend vector2f min(const vector2f& vec1, const vector2f& vec2);
  friend vector2f max(const vector2f& vec1, const vector2f& vec2);
  friend vector2f floor(const vector2f& vec);
  friend vector2f ceil(const vector2f& vec);
  friend vector2f round(const vector2f& vec);
  friend vector2f fmod(const vector2f& vec, float val);
  friend vector2f fmin(const vector2f& vec, float val);
  friend vector2f fmax(const vector2f& vec, float val);
};

#endif  // VECTOR2F_HPP
