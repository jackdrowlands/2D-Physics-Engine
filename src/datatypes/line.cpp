#include "../../include/datatypes/line.hpp"

line::line(vector2d from, vector2d to) {
  this->from = from;
  this->to = to;
}

line::~line() {}

vector2d line::getFrom() { return this->from; }

vector2d line::getTo() { return this->to; }