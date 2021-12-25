/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

// The original file belongs to MSCKF_VIO (https://github.com/KumarRobotics/msckf_vio/)
// Some changes have been made to use it in livox_slam_ware

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <cmath>
#include <Eigen/Dense>

namespace livox_slam_ware {

/*
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w) {
  Eigen::Matrix3d w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
inline void quaternionNormalize(Eigen::Vector4d& q) {
  double norm = q.norm();
  q = q / norm;
  return;
}

/*
 * @brief Perform q1 * q2.
 *  
 *    Format of q1 and q2 is as [x,y,z,w]
 */
inline Eigen::Vector4d quaternionMultiplication(
    const Eigen::Vector4d& q1,
    const Eigen::Vector4d& q2) {
  Eigen::Matrix4d L;

  // QXC: Hamilton
  L(0, 0) =  q1(3); L(0, 1) = -q1(2); L(0, 2) =  q1(1); L(0, 3) =  q1(0);
  L(1, 0) =  q1(2); L(1, 1) =  q1(3); L(1, 2) = -q1(0); L(1, 3) =  q1(1);
  L(2, 0) = -q1(1); L(2, 1) =  q1(0); L(2, 2) =  q1(3); L(2, 3) =  q1(2);
  L(3, 0) = -q1(0); L(3, 1) = -q1(1); L(3, 2) = -q1(2); L(3, 3) =  q1(3);

  Eigen::Vector4d q = L * q2;
  quaternionNormalize(q);
  return q;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *    full quaternion.
 * @note This function is useful to convert delta quaternion
 *    which is usually a 3x1 vector to a full quaternion.
 *    For more details, check Section 3.2 "Kalman Filter Update" in
 *    "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for quaternion Algebra".
 */
inline Eigen::Vector4d smallAngleQuaternion(
    const Eigen::Vector3d& dtheta) {

  Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Vector4d q;
  double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.head<3>() = dq;
    q(3) = std::sqrt(1-dq_square_norm);
  } else {
    q.head<3>() = dq;
    q(3) = 1;
    q = q / std::sqrt(1+dq_square_norm);
  }

  return q;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *    full quaternion.
 * @note This function is useful to convert delta quaternion
 *    which is usually a 3x1 vector to a full quaternion.
 *    For more details, check Section 3.2 "Kalman Filter Update" in
 *    "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for quaternion Algebra".
 */
inline Eigen::Quaterniond getSmallAngleQuaternion(
    const Eigen::Vector3d& dtheta) {

  Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Quaterniond q;
  double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.x() = dq(0);
    q.y() = dq(1);
    q.z() = dq(2);
    q.w() = std::sqrt(1-dq_square_norm);
  } else {
    q.x() = dq(0);
    q.y() = dq(1);
    q.z() = dq(2);
    q.w() = 1;
    q.normalize();
  }

  return q;
}

/*
 * @brief Convert a quaternion to the corresponding rotation matrix
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
inline Eigen::Matrix3d quaternionToRotation(
    const Eigen::Vector4d& q) {
  // QXC: Hamilton
  const double& qw = q(3);
  const double& qx = q(0);
  const double& qy = q(1);
  const double& qz = q(2);
  Eigen::Matrix3d R;
  R(0, 0) = 1-2*(qy*qy+qz*qz);  R(0, 1) =   2*(qx*qy-qw*qz);  R(0, 2) =   2*(qx*qz+qw*qy);
  R(1, 0) =   2*(qx*qy+qw*qz);  R(1, 1) = 1-2*(qx*qx+qz*qz);  R(1, 2) =   2*(qy*qz-qw*qx);
  R(2, 0) =   2*(qx*qz-qw*qy);  R(2, 1) =   2*(qy*qz+qw*qx);  R(2, 2) = 1-2*(qx*qx+qy*qy);

  return R;
}

/*
 * @brief Convert a rotation matrix to a quaternion.
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
inline Eigen::Vector4d rotationToQuaternion(
    const Eigen::Matrix3d& R) {
  Eigen::Vector4d score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = R.trace();

  int max_row = 0, max_col = 0;
  score.maxCoeff(&max_row, &max_col);

  Eigen::Vector4d q = Eigen::Vector4d::Zero();

  // QXC: Hamilton
  if (max_row == 0) {
    q(0) = std::sqrt(1+2*R(0, 0)-R.trace()) / 2.0;
    q(1) = (R(0, 1)+R(1, 0)) / (4*q(0));
    q(2) = (R(0, 2)+R(2, 0)) / (4*q(0));
    q(3) = (R(2, 1)-R(1, 2)) / (4*q(0));
  } else if (max_row == 1) {
    q(1) = std::sqrt(1+2*R(1, 1)-R.trace()) / 2.0;
    q(0) = (R(0, 1)+R(1, 0)) / (4*q(1));
    q(2) = (R(1, 2)+R(2, 1)) / (4*q(1));
    q(3) = (R(0, 2)-R(2, 0)) / (4*q(1));
  } else if (max_row == 2) {
    q(2) = std::sqrt(1+2*R(2, 2)-R.trace()) / 2.0;
    q(0) = (R(0, 2)+R(2, 0)) / (4*q(2));
    q(1) = (R(1, 2)+R(2, 1)) / (4*q(2));
    q(3) = (R(1, 0)-R(0, 1)) / (4*q(2));
  } else {
    q(3) = std::sqrt(1+R.trace()) / 2.0;
    q(0) = (R(2, 1)-R(1, 2)) / (4*q(3));
    q(1) = (R(0, 2)-R(2, 0)) / (4*q(3));
    q(2) = (R(1, 0)-R(0, 1)) / (4*q(3));
  }

  if (q(3) < 0) q = -q;
  quaternionNormalize(q);
  return q;
}

/*
 * @brief Convert a rotation matrix to a eulerAngles.
 */
template <typename T>
void R2ypr(const Eigen::Matrix<T, 3, 3>& R, Eigen::Matrix<T, 3, 1> &ypr) {
  Eigen::Matrix<T, 3, 1> n = R.col(0);
  Eigen::Matrix<T, 3, 1> o = R.col(1);
  Eigen::Matrix<T, 3, 1> a = R.col(2);

  T y = atan2(n(1), n(0));
  T p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  T r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;
}
/*
 * @brief Calculate the error between the two planes.
 */
template <typename T>
Eigen::Matrix<T, 3, 1> ominus(const Eigen::Matrix<T, 4, 1>& ground_plane_coeff, Eigen::Matrix<T, 4, 1> &local_ground_plane) {
  Eigen::Matrix<T, 3, 1> n_ground_plane_coeff = ground_plane_coeff.template segment<3>(0);
  Eigen::Matrix<T, 3, 1> n_local_ground_plane = local_ground_plane.template segment<3>(0);
  Eigen::Matrix<float, 3, 1> x(1, 0, 0);
  Eigen::Matrix<T, 3, 1> normal_x = x.template cast<T>();
  Eigen::Matrix<T, 3, 3> R = Eigen::Quaternion<T>::FromTwoVectors(n_local_ground_plane, normal_x).toRotationMatrix();
  Eigen::Matrix<T, 3, 1> n_ground_plane_coeff_after_rotation = R * n_ground_plane_coeff;
  Eigen::Matrix<T, 3, 1> result;
  result(0) = atan2(n_ground_plane_coeff_after_rotation(1), n_ground_plane_coeff_after_rotation(0));
  result(1) = atan2(n_ground_plane_coeff_after_rotation(2), n_ground_plane_coeff_after_rotation.template head<2>().norm());
  result(2) = -local_ground_plane(3) + ground_plane_coeff(3);
  return result;
}
} // end namespace livox_slam_ware

#endif // MATH_UTILS_HPP
