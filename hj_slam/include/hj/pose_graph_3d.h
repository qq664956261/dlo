// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)

#pragma once
#include <istream>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"
namespace ceres
{
  namespace vn_fusion
  {

    struct Pose3d
    {
      Eigen::Vector3d p;
      Eigen::Quaterniond q;

      // The name of the data type in the g2o file format.
      static std::string name()
      {
        return "VERTEX_SE3:QUAT";
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline Pose3d Mul(const Pose3d &p_a, const Pose3d &p_b)
    {
      Pose3d p;
      p.q = p_a.q * p_b.q;
      p.p = p_a.q * p_b.p + p_a.p;
      return p;
    }

    inline Pose3d Inv(const Pose3d &p)
    {
      Pose3d p_inv;
      p_inv.q = p.q.inverse();
      p_inv.p = p_inv.q * (-p.p);
      return p_inv;
    }

    inline Pose3d Between(const Pose3d &p_a, const Pose3d &p_b)
    {
      Pose3d p_ab;
      auto q_a_inv = p_a.q.inverse();
      p_ab.q = q_a_inv * p_b.q;
      p_ab.p = q_a_inv * (p_b.p - p_a.p);
      return p_ab;
    }

    template <typename Scalar>
    Pose3d Mat4ToPose3d(typename Eigen::Matrix<Scalar, 4, 4> &T)
    {
      Pose3d pose;
      Eigen::Quaternion<Scalar> q(T.template topLeftCorner<3, 3>());
      pose.q = Eigen::Quaterniond((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
      pose.p = Eigen::Vector3d((double)T(3, 0), (double)T(3, 1), (double)T(3, 2));
      return pose;
    }

    inline std::istream &operator>>(std::istream &input, Pose3d &pose)
    {
      input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
          pose.q.y() >> pose.q.z() >> pose.q.w();
      // Normalize the quaternion to account for precision loss due to
      // serialization.
      pose.q.normalize();
      return input;
    }

    typedef std::map<int, Pose3d, std::less<int>,
                     Eigen::aligned_allocator<std::pair<const int, Pose3d>>>
        MapOfPoses;

    typedef std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>>
        VecOfPoses;
    // The constraint between two vertices in the pose graph. The constraint is the
    // transformation from vertex id_begin to vertex id_end.
    struct Constraint3d
    {
      int id_begin;
      int id_end;

      // The transformation that represents the pose of the end frame E w.r.t. the
      // begin frame B. In other words, it transforms a vector in the E frame to
      // the B frame.
      Pose3d t_be;

      // The inverse of the covariance matrix for the measurement. The order of the
      // entries are x, y, z, delta orientation.
      Eigen::Matrix<double, 6, 6> information;

      // The name of the data type in the g2o file format.
      static std::string name()
      {
        return "EDGE_SE3:QUAT";
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline std::istream &operator>>(std::istream &input, Constraint3d &constraint)
    {
      Pose3d &t_be = constraint.t_be;
      input >> constraint.id_begin >> constraint.id_end >> t_be;

      for (int i = 0; i < 6 && input.good(); ++i)
      {
        for (int j = i; j < 6 && input.good(); ++j)
        {
          input >> constraint.information(i, j);
          if (i != j)
          {
            constraint.information(j, i) = constraint.information(i, j);
          }
        }
      }
      return input;
    }

    typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>
        VectorOfConstraints;

    // Computes the error term for two poses that have a relative pose measurement
    // between them. Let the hat variables be the measurement. We have two poses x_a
    // and x_b. Through sensor measurements we can measure the transformation of
    // frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
    // between the current estimate of the poses and the measurement.
    //
    // In this formulation, we have chosen to represent the rigid transformation as
    // a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
    // [x, y, z, w].

    // The estimated measurement is:
    //      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
    //             [ q_ab ]    [ q_a^{-1] * q_b         ]
    //
    // where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
    // quaternion. Now we can compute an error metric between the estimated and
    // measurement transformation. For the orientation error, we will use the
    // standard multiplicative error resulting in:
    //
    //   error = [ p_ab - \hat{p}_ab                 ]
    //           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
    //
    // where Vec(*) returns the vector (imaginary) part of the quaternion. Since
    // the measurement has an uncertainty associated with how accurate it is, we
    // will weight the errors by the square root of the measurement information
    // matrix:
    //
    //   residuals = I^{1/2) * error
    // where I is the information matrix which is the inverse of the covariance.
    class PoseGraph3dErrorTerm
    {
    public:
      PoseGraph3dErrorTerm(const Pose3d &t_ab_measured,
                           const Eigen::Matrix<double, 6, 6> &sqrt_information)
          : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

      template <typename T>
      bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                      const T *const p_b_ptr, const T *const q_b_ptr,
                      T *residuals_ptr) const
      {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
            t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
            p_ab_estimated - t_ab_measured_.p.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
      }

      static ceres::CostFunction *Create(
          const Pose3d &t_ab_measured,
          const Eigen::Matrix<double, 6, 6> &sqrt_information)
      {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
            new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      // The measurement for the position of B relative to A in the A frame.
      const Pose3d t_ab_measured_;
      // The square root of the measurement information matrix.
      const Eigen::Matrix<double, 6, 6> sqrt_information_;
    };

    void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                                  MapOfPoses *poses, ceres::Problem *problem);

    // Returns true if the solve was successful.
    bool SolveOptimizationProblem(ceres::Problem *problem);

  } // namespace vn_fusion
} // namespace ceres
