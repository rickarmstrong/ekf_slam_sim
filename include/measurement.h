// Copyright (c) 2025 Richard Armstrong
#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP
#include "types.hpp"

namespace ekf_localizer
{
  class Measurement final
  {
  public:
    // Prevent default construction.
    Measurement(double x, double y, unsigned id): x(x), y(y), id(id) {}
    explicit Measurement(const TagDetection& td);
    double x;
    double y;
    unsigned id;
  };
}  // namespace ekf_localizer
#endif //MEASUREMENT_HPP