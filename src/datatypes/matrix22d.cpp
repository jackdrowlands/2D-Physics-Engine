#include "../../include/datatypes/matrix22d.hpp"

// Default constructor, initializes to identity matrix
matrix22d::matrix22d() {
  data[0][0] = 1;
  data[0][1] = 0;
  data[1][0] = 0;
  data[1][1] = 1;
}

// Constructor that initializes matrix with given values
matrix22d::matrix22d(double a, double b, double c, double d) {
  data[0][0] = a;
  data[0][1] = b;
  data[1][0] = c;
  data[1][1] = d;
}

// Constructor that initializes matrix with a rotation of 'radians' radians
matrix22d::matrix22d(double radians) {
  double c = cos(radians);
  double s = sin(radians);
  data[0][0] = c;
  data[0][1] = -s;
  data[1][0] = s;
  data[1][1] = c;
}

// Destructor
matrix22d::~matrix22d() {}

// Function to calculate and return the determinant of the matrix
double matrix22d::getDeterminant() {
  return data[0][0] * data[1][1] - data[0][1] * data[1][0];
}

// Function to calculate and return the inverse of the matrix
matrix22d matrix22d::getInverse() {
  double det = getDeterminant();
  if (det == 0) {
    return matrix22d();
  }
  double invDet = 1 / det;
  return matrix22d(data[1][1] * invDet, -data[0][1] * invDet,
                   -data[1][0] * invDet, data[0][0] * invDet);
}

// Function to invert the matrix in place
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

// Function to calculate and return the transpose of the matrix
matrix22d matrix22d::getTranspose() {
  return matrix22d(data[0][0], data[1][0], data[0][1], data[1][1]);
}

// Function to transpose the matrix in place
void matrix22d::transpose() {
  double temp = data[0][1];
  data[0][1] = data[1][0];
  data[1][0] = temp;
}

// Overloaded operator* for matrix multiplication
matrix22d matrix22d::operator*(matrix22d& m) {
  return matrix22d(data[0][0] * m[0][0] + data[0][1] * m[1][0],
                   data[0][0] * m[0][1] + data[0][1] * m[1][1],
                   data[1][0] * m[0][0] + data[1][1] * m[1][0],
                   data[1][0] * m[0][1] + data[1][1] * m[1][1]);
}

// Overloaded operator* for scalar multiplication
matrix22d matrix22d::operator*(double s) {
  return matrix22d(data[0][0] * s, data[0][1] * s, data[1][0] * s,
                   data[1][1] * s);
}

// Overloaded operator+ for matrix addition
matrix22d matrix22d::operator+(matrix22d& m) {
  return matrix22d(data[0][0] + m[0][0], data[0][1] + m[0][1],
                   data[1][0] + m[1][0], data[1][1] + m[1][1]);
}

// Overloaded operator- for matrix subtraction
matrix22d matrix22d::operator-(matrix22d& m) {
  return matrix22d(data[0][0] - m[0][0], data[0][1] - m[0][1],
                   data[1][0] - m[1][0], data[1][1] - m[1][1]);
}

// Overloaded operator[] to access matrix elements
double* matrix22d::operator[](int i) { return data[i]; }

// Overloaded operator[] to access matrix elements (const version)
const double* matrix22d::operator[](int i) const { return data[i]; }

// Function to calculate and return the absolute value of the matrix
matrix22d abs(const matrix22d& m) {
  return matrix22d(abs(m[0][0]), abs(m[0][1]), abs(m[1][0]), abs(m[1][1]));
}