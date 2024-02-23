#include "../../include/datatypes/line.hpp"

/**
 * @brief Constructs a line object with the given starting and ending points.
 *
 * @param from The starting point of the line.
 * @param to The ending point of the line.
 */
line::line(vector2d from, vector2d to) {
  this->from = from;
  this->to = to;
}

line::~line() {}

vector2d line::getFrom() { return this->from; }

vector2d line::getTo() { return this->to; }