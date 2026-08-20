/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <vins/factor/pose_local_parameterization.h>

bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq =
      Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}
bool PoseLocalParameterization::PlusJacobian(const double *x,
                                             double *jacobian) const {
  (void)x;
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x,
                                      double *y_minus_x) const {
  Eigen::Map<const Eigen::Vector3d> p_y(y);
  Eigen::Map<const Eigen::Vector3d> p_x(x);
  Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);
  Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);

  delta.head<3>() = p_y - p_x;
  Eigen::Quaterniond dq = q_x.conjugate() * q_y;
  if (dq.w() < 0.0) {
    dq.coeffs() *= -1.0;
  }
  const double vector_norm = dq.vec().norm();
  if (vector_norm > 0.0) {
    delta.tail<3>() =
        (2.0 * std::atan2(vector_norm, dq.w()) / vector_norm) * dq.vec();
  } else {
    delta.tail<3>().setZero();
  }
  return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x,
                                              double *jacobian) const {
  (void)x;
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
  j.leftCols<6>().setIdentity();
  j.rightCols<1>().setZero();
  return true;
}
