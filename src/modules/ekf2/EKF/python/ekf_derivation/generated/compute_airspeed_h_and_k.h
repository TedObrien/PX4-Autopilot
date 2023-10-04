// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_airspeed_h_and_k
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix24_24
 *     innov_var: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     H: Matrix24_1
 *     K: Matrix24_1
 */
template <typename Scalar>
void ComputeAirspeedHAndK(const matrix::Matrix<Scalar, 24, 1>& state,
                          const matrix::Matrix<Scalar, 24, 24>& P, const Scalar innov_var,
                          const Scalar epsilon, matrix::Matrix<Scalar, 24, 1>* const H = nullptr,
                          matrix::Matrix<Scalar, 24, 1>* const K = nullptr) {
  // Total ops: 256

  // Input arrays

  // Intermediate terms (7)
  const Scalar _tmp0 = -state(23, 0) + state(5, 0);
  const Scalar _tmp1 = -state(22, 0) + state(4, 0);
  const Scalar _tmp2 = std::pow(Scalar(std::pow(_tmp0, Scalar(2)) + std::pow(_tmp1, Scalar(2)) +
                                       epsilon + std::pow(state(6, 0), Scalar(2))),
                                Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp3 = _tmp1 * _tmp2;
  const Scalar _tmp4 = _tmp0 * _tmp2;
  const Scalar _tmp5 = _tmp2 * state(6, 0);
  const Scalar _tmp6 = Scalar(1.0) / (math::max<Scalar>(epsilon, innov_var));

  // Output terms (2)
  if (H != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _h = (*H);

    _h.setZero();

    _h(4, 0) = _tmp3;
    _h(5, 0) = _tmp4;
    _h(6, 0) = _tmp5;
    _h(22, 0) = -_tmp3;
    _h(23, 0) = -_tmp4;
  }

  if (K != nullptr) {
    matrix::Matrix<Scalar, 24, 1>& _k = (*K);

    _k(0, 0) = _tmp6 * (-P(0, 22) * _tmp3 - P(0, 23) * _tmp4 + P(0, 4) * _tmp3 + P(0, 5) * _tmp4 +
                        P(0, 6) * _tmp5);
    _k(1, 0) = _tmp6 * (-P(1, 22) * _tmp3 - P(1, 23) * _tmp4 + P(1, 4) * _tmp3 + P(1, 5) * _tmp4 +
                        P(1, 6) * _tmp5);
    _k(2, 0) = _tmp6 * (-P(2, 22) * _tmp3 - P(2, 23) * _tmp4 + P(2, 4) * _tmp3 + P(2, 5) * _tmp4 +
                        P(2, 6) * _tmp5);
    _k(3, 0) = _tmp6 * (-P(3, 22) * _tmp3 - P(3, 23) * _tmp4 + P(3, 4) * _tmp3 + P(3, 5) * _tmp4 +
                        P(3, 6) * _tmp5);
    _k(4, 0) = _tmp6 * (-P(4, 22) * _tmp3 - P(4, 23) * _tmp4 + P(4, 4) * _tmp3 + P(4, 5) * _tmp4 +
                        P(4, 6) * _tmp5);
    _k(5, 0) = _tmp6 * (-P(5, 22) * _tmp3 - P(5, 23) * _tmp4 + P(5, 4) * _tmp3 + P(5, 5) * _tmp4 +
                        P(5, 6) * _tmp5);
    _k(6, 0) = _tmp6 * (-P(6, 22) * _tmp3 - P(6, 23) * _tmp4 + P(6, 4) * _tmp3 + P(6, 5) * _tmp4 +
                        P(6, 6) * _tmp5);
    _k(7, 0) = _tmp6 * (-P(7, 22) * _tmp3 - P(7, 23) * _tmp4 + P(7, 4) * _tmp3 + P(7, 5) * _tmp4 +
                        P(7, 6) * _tmp5);
    _k(8, 0) = _tmp6 * (-P(8, 22) * _tmp3 - P(8, 23) * _tmp4 + P(8, 4) * _tmp3 + P(8, 5) * _tmp4 +
                        P(8, 6) * _tmp5);
    _k(9, 0) = _tmp6 * (-P(9, 22) * _tmp3 - P(9, 23) * _tmp4 + P(9, 4) * _tmp3 + P(9, 5) * _tmp4 +
                        P(9, 6) * _tmp5);
    _k(10, 0) = _tmp6 * (-P(10, 22) * _tmp3 - P(10, 23) * _tmp4 + P(10, 4) * _tmp3 +
                         P(10, 5) * _tmp4 + P(10, 6) * _tmp5);
    _k(11, 0) = _tmp6 * (-P(11, 22) * _tmp3 - P(11, 23) * _tmp4 + P(11, 4) * _tmp3 +
                         P(11, 5) * _tmp4 + P(11, 6) * _tmp5);
    _k(12, 0) = _tmp6 * (-P(12, 22) * _tmp3 - P(12, 23) * _tmp4 + P(12, 4) * _tmp3 +
                         P(12, 5) * _tmp4 + P(12, 6) * _tmp5);
    _k(13, 0) = _tmp6 * (-P(13, 22) * _tmp3 - P(13, 23) * _tmp4 + P(13, 4) * _tmp3 +
                         P(13, 5) * _tmp4 + P(13, 6) * _tmp5);
    _k(14, 0) = _tmp6 * (-P(14, 22) * _tmp3 - P(14, 23) * _tmp4 + P(14, 4) * _tmp3 +
                         P(14, 5) * _tmp4 + P(14, 6) * _tmp5);
    _k(15, 0) = _tmp6 * (-P(15, 22) * _tmp3 - P(15, 23) * _tmp4 + P(15, 4) * _tmp3 +
                         P(15, 5) * _tmp4 + P(15, 6) * _tmp5);
    _k(16, 0) = _tmp6 * (-P(16, 22) * _tmp3 - P(16, 23) * _tmp4 + P(16, 4) * _tmp3 +
                         P(16, 5) * _tmp4 + P(16, 6) * _tmp5);
    _k(17, 0) = _tmp6 * (-P(17, 22) * _tmp3 - P(17, 23) * _tmp4 + P(17, 4) * _tmp3 +
                         P(17, 5) * _tmp4 + P(17, 6) * _tmp5);
    _k(18, 0) = _tmp6 * (-P(18, 22) * _tmp3 - P(18, 23) * _tmp4 + P(18, 4) * _tmp3 +
                         P(18, 5) * _tmp4 + P(18, 6) * _tmp5);
    _k(19, 0) = _tmp6 * (-P(19, 22) * _tmp3 - P(19, 23) * _tmp4 + P(19, 4) * _tmp3 +
                         P(19, 5) * _tmp4 + P(19, 6) * _tmp5);
    _k(20, 0) = _tmp6 * (-P(20, 22) * _tmp3 - P(20, 23) * _tmp4 + P(20, 4) * _tmp3 +
                         P(20, 5) * _tmp4 + P(20, 6) * _tmp5);
    _k(21, 0) = _tmp6 * (-P(21, 22) * _tmp3 - P(21, 23) * _tmp4 + P(21, 4) * _tmp3 +
                         P(21, 5) * _tmp4 + P(21, 6) * _tmp5);
    _k(22, 0) = _tmp6 * (-P(22, 22) * _tmp3 - P(22, 23) * _tmp4 + P(22, 4) * _tmp3 +
                         P(22, 5) * _tmp4 + P(22, 6) * _tmp5);
    _k(23, 0) = _tmp6 * (-P(23, 22) * _tmp3 - P(23, 23) * _tmp4 + P(23, 4) * _tmp3 +
                         P(23, 5) * _tmp4 + P(23, 6) * _tmp5);
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
