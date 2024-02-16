#ifndef LINE_HPP
#define LINE_HPP

#include "include/datatypes/vector2d.hpp"

class line {
 private:
  vector2d from;
  vector2d to;

 public:
  line(vector2d from, vector2d to);
  ~line();
  vector2d getFrom();
  vector2d getTo();
};

#endif  // LINE_HPP
