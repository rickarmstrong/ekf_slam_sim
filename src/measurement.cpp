// Copyright (c) 2025 Rick Armstrong
#include "measurement.h"

namespace ekf_localizer
{
  Measurement::Measurement(const TagDetection& td)
  {
    if (td.id.size() != 1)
    {
      throw std::invalid_argument("TagDetection must have exactly one id. Tag bundles and empty tag ids "
                                  "are not supported.");
    }
    x = td.pose.pose.pose.position.x;
    y = td.pose.pose.pose.position.y;
    id = td.id[0];
  }
}  // namespace ekf_localizer
