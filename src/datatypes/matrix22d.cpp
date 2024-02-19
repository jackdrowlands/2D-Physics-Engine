#include "../../include/datatypes/matrix22d.hpp"

matrix22d::matrix22d() {
  data[0][0] = 1;
  data[0][1] = 0;
  data[1][0] = 0;
  data[1][1] = 1;
}

matrix22d::matrix22d(double a, double b, double c, double d) {
  data[0][0] = a;
  data[0][1] = b;
  data[1][0] = c;
  data[1][1] = d;
}

matrix22d::matrix22d(double radians) {
  double c = cos(radians);
  double s = sin(radians);
  data[0][0] = c;
  data[0][1] = -s;
  data[1][0] = s;
  data[1][1] = c;
}

matrix22d::~matrix22d() {}

double matrix22d::getDeterminant() {
  return data[0][0] * data[1][1] - data[0][1] * data[1][0];
}

matrix22d matrix22d::getInverse() {
  double det = getDeterminant();
  if (det == 0) {
    return matrix22d();
  }
  double invDet = 1 / det;
  return matrix22d(data[1][1] * invDet, -data[0][1] * invDet,
                   -data[1][0] * invDet, data[0][0] * invDet);
}

void matrix22d::invert() {
  double det = getDeterminant();
  if (det == 0) {
    return;
  }
  double invDet = 1 / det;
  double temp = data[0][0];
  data[0][0] = data[1][1] * invDet;
  data[0][1] = -data[0][1] * invDet;
  data[1][0] = -data[1][0] * invDet;
  data[1][1] = temp * invDet;
}

matrix22d matrix22d::getTranspose() {
  return matrix22d(data[0][0], data[1][0], data[0][1], data[1][1]);
}

void matrix22d::transpose() {
  double temp = data[0][1];
  data[0][1] = data[1][0];
  data[1][0] = temp;
}

matrix22d matrix22d::operator*(matrix22d& m) {
  return matrix22d(data[0][0] * m[0][0] + data[0][1] * m[1][0],
                   data[0][0] * m[0][1] + data[0][1] * m[1][1],
                   data[1][0] * m[0][0] + data[1][1] * m[1][0],
                   data[1][0] * m[0][1] + data[1][1] * m[1][1]);
}

matrix22d matrix22d::operator*(double s) {
  return matrix22d(data[0][0] * s, data[0][1] * s, data[1][0] * s,
                   data[1][1] * s);
}

matrix22d matrix22d::operator+(matrix22d& m) {
  return matrix22d(data[0][0] + m[0][0], data[0][1] + m[0][1],
                   data[1][0] + m[1][0], data[1][1] + m[1][1]);
}

matrix22d matrix22d::operator-(matrix22d& m) {
  return matrix22d(data[0][0] - m[0][0], data[0][1] - m[0][1],
                   data[1][0] - m[1][0], data[1][1] - m[1][1]);
}

double* matrix22d::operator[](int i) { return data[i]; }

const double* matrix22d::operator[](int i) const { return data[i]; }

matrix22d abs(const matrix22d& m) {
  return matrix22d(abs(m[0][0]), abs(m[0][1]), abs(m[1][0]), abs(m[1][1]));
}