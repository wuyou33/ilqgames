/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Polyline2 class for piecewise linear paths in 2D.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/geometry/line_segment2.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/utils/types.h>

#include <glog/logging.h>

namespace ilqgames {

Polyline2::Polyline2(const PointList2& points) : length_(0.0) {
  CHECK_GT(points.size(), 1);

  // Parse into list of line segents.
  for (size_t ii = 1; ii < points.size(); ii++) {
    segments_.emplace_back(points[ii - 1], points[ii]);
    length_ += segments_.back().Length();
  }
}

void Polyline2::AddPoint(const Point2& point) {
  segments_.emplace_back(segments_.back().SecondPoint(), point);
  length_ += segments_.back().Length();
}

Point2 Polyline2::ClosestPoint(const Point2& query, bool* is_vertex,
                               LineSegment2* segment,
                               float* signed_squared_distance) const {
  // Walk along each line segment and remember which was closest.
  float closest_signed_squared_distance = constants::kInfinity;
  Point2 closest_point;

  float current_signed_squared_distance;
  for (const auto& s : segments_) {
    bool is_endpoint;
    const Point2 current_point =
        s.ClosestPoint(query, &is_endpoint, &current_signed_squared_distance);

    if (std::abs(current_signed_squared_distance) <
        std::abs(closest_signed_squared_distance)) {
      closest_signed_squared_distance = current_signed_squared_distance;
      closest_point = current_point;

      if (is_vertex) *is_vertex = is_endpoint;
      if (segment) *segment = s;
    }
  }

  // Maybe set signed_squared_distance.
  if (signed_squared_distance)
    *signed_squared_distance = closest_signed_squared_distance;

  return closest_point;
}

}  // namespace ilqgames
