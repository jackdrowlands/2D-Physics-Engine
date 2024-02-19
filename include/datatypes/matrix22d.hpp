#ifndef MATRIX22D_HPP
#define MATRIX22D_HPP

#include <cmath>

class matrix22d {
 private:
  double data[2][2];

 public:
  matrix22d();
  matrix22d(double a, double b, double c, double d);
  matrix22d(double radians);
  ~matrix22d();
  double getDeterminant();
  matrix22d getInverse();
  void invert();
  matrix22d getTranspose();
  void transpose();
  matrix22d operator*(matrix22d& m);
  matrix22d operator*(double s);
  matrix22d operator+(matrix22d& m);
  matrix22d operator-(matrix22d& m);
  double* operator[](int i);
  const double* operator[](int i) const;
  friend matrix22d abs(const matrix22d& m);
};

#endif  // MATRIX22D_HPP