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
// Container to store a single player's time-indexed value function.
//
// Notation is taken from Basar and Olsder, Corollary 6.1.
// -- zetas are the linear terms (i.e., linear in state x)
// -- Zs are the quadratic terms
// i.e. lambda (costate) = Z delta_x + zeta
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_VALUE_FUNCTION_H
#define ILQGAMES_UTILS_VALUE_FUNCTION_H

#include <ilqgames/utils/types.h>

#include <glog/logging.h>
#include <vector>

namespace ilqgames {

struct ValueFunction {
  std::vector<MatrixXf> Zs;
  std::vector<VectorXf> zetas;

  // Preallocate memory during construction.
  ValueFunction(size_t horizon, Dimension xdim) : Zs(horizon), zetas(horizon) {
    for (size_t ii = 0; ii < horizon; ii++) {
      Zs[ii] = MatrixXf::Zero(xdim, xdim);
      zetas[ii] = VectorXf::Zero(xdim);
    }
  }

  // Compute costate given time index and delta x.
  VectorXf Costate(size_t time_index, const VectorXf& delta_x,
                   const VectorXf& lambda_ref) const {
    // NOTE: did we get the signs right?
    return lambda_ref + Zs[time_index] * delta_x + zetas[time_index];
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};  // struct ValueFunction

}  // namespace ilqgames

#endif
