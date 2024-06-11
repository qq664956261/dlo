#include <stdio.h>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <cmath>
#include "hj/pose_graph_3d.h"
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include "hj/PointReader.hh"

using namespace std;
using namespace ceres::vn_fusion;
enum MPEMsg {
    UnDefined_Err = -1,
    CalibExtrinsicSuccess = 0,
    LessKFs_Err = 1,
    LargeKFGap_Err = 2,
    Tag_Err = 3
};

class ExtrinsicErrorTerm
{
public:
  ExtrinsicErrorTerm();
  ~ExtrinsicErrorTerm();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ExtrinsicErrorTerm> Ptr;
  ExtrinsicErrorTerm(Pose3d delta_odom, Pose3d delta_laser,
                     Eigen::Matrix<double, 6, 6> sqrt_information)
      : delta_odom_(std::move(delta_odom)), delta_laser_(std::move(delta_laser)),
        sqrt_information_(std::move(sqrt_information)) {}

  template <typename T>
  bool operator()(const T *const p_ptr, const T *const q_ptr, T *residuals_ptr) const
  {
    Eigen::Matrix<T, 3, 1> p_agv_laser;
    p_agv_laser[0] = p_ptr[0];
    p_agv_laser[1] = p_ptr[1];
    p_agv_laser[2] = T(0);
    Eigen::Map<const Eigen::Quaternion<T>> q_agv_laser(q_ptr);
    Eigen::Quaternion<T> q_laser_agv = q_agv_laser.conjugate();
    Eigen::Matrix<T, 3, 1> p_laser_agv = q_laser_agv * (-p_agv_laser);

    //Eigen::Quaternion<T> q_laser_laser = q_laser_agv *
    //                                     delta_odom_.q.template cast<T>() * q_agv_laser;
    //Eigen::Matrix<T, 3, 1> p_laser_laser = q_laser_agv *
    //                                           (delta_odom_.q.template cast<T>() * p_agv_laser + delta_odom_.p.template cast<T>()) +
    //                                       p_laser_agv;
    Eigen::Quaternion<T> q_agv_agv = q_agv_laser *
                                         delta_laser_.q.template cast<T>() * q_laser_agv;
    Eigen::Matrix<T, 3, 1> p_agv_agv = q_agv_laser *
                                               (delta_laser_.q.template cast<T>() * p_laser_agv + delta_laser_.p.template cast<T>()) +
                                           p_agv_laser;

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        delta_odom_.p.template cast<T>() - p_agv_agv;
	Eigen::Quaternion<T> delta_q = q_agv_agv.conjugate() * delta_odom_.q.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }

  static ceres::CostFunction *Create(
      const Pose3d &delta_odom, const Pose3d &delta_laser,
      const Eigen::Matrix<double, 6, 6> &sqrt_information)
  {
    return new ceres::AutoDiffCostFunction<ExtrinsicErrorTerm, 6, 2, 4>(
        new ExtrinsicErrorTerm(delta_odom, delta_laser, sqrt_information));
  }
  template <typename Scalar>
  inline Eigen::Matrix<Scalar, 3, 1> R2ypr(const Eigen::Matrix<Scalar, 3, 3>& R)
  {
      Eigen::Matrix<Scalar, 3, 1> n = R.col(0);
      Eigen::Matrix<Scalar, 3, 1> o = R.col(1);
      Eigen::Matrix<Scalar, 3, 1> a = R.col(2);

      Eigen::Matrix<Scalar, 3, 1> ypr(3);
      double y = atan2(n(1), n(0));
      double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
      double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
      ypr(0) = y;
      ypr(1) = p;
      ypr(2) = r;

      return ypr;
  }
  template <typename Scalar>
  inline Eigen::Matrix<Scalar, 3, 3> ypr2R(const Eigen::Matrix<Scalar, 3, 1> ypr)
  {


      //Scalar y = ypr(0) / 180.0 * M_PI;
      //Scalar p = ypr(1) / 180.0 * M_PI;
      //Scalar r = ypr(2) / 180.0 * M_PI;
      Scalar y = ypr(0);
      Scalar p = ypr(1);
      Scalar r = ypr(2);
      Eigen::Matrix<Scalar, 3, 3> Rz;
      Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;

      Eigen::Matrix<Scalar, 3, 3> Ry;
      Ry << cos(p), 0., sin(p),
          0., 1., 0.,
          -sin(p), 0., cos(p);

      Eigen::Matrix<Scalar, 3, 3> Rx;
      Rx << 1., 0., 0.,
          0., cos(r), -sin(r),
          0., sin(r), cos(r);

      return Rz * Ry * Rx;
  }
  template <typename Scalar>
  inline Eigen::Matrix<Scalar, 3, 1> getYpr(const Eigen::Matrix<Scalar, 3, 3> &rot_mat)
  {
    Eigen::Matrix<Scalar, 3, 1> ypr;
    Scalar s2 = -rot_mat(2, 0);
    // With sqrt, the range of pitch decreases from [-pi:pi] to [-pi/2:pi/2]
    Scalar c2 = std::sqrt(rot_mat(0, 0) * rot_mat(0, 0) + rot_mat(1, 0) * rot_mat(1, 0));
    if (c2 > 1e-6)
    {
      ypr[0] = std::atan2(rot_mat(1, 0), rot_mat(0, 0));
      ypr[1] = std::atan2(s2, c2);
      ypr[2] = std::atan2(rot_mat(2, 1), rot_mat(2, 2));
    }
    else // singularity case
    {
      ypr[0] = 0;
      ypr[1] = std::atan2(s2, c2);
      ypr[2] = std::atan2(rot_mat(0, 1), rot_mat(0, 2));
    }
    return ypr;
  }
  inline Eigen::Quaterniond ypr2Quat(double yaw, double pitch, double roll)
  {
    Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd y(yaw, Eigen::Vector3d::UnitZ());
    return y * p * r;
  }

  int runCalibExtrinsic(VecOfPoses odom_poses, VecOfPoses laser_poses, double &x, double &y, double &z, double &yaw, double &pitch, double &roll);
  int runCalibExtrinsicRansac(VecOfPoses odom_poses, VecOfPoses laser_poses, double &x, double &y, double &z, double &yaw, double &pitch, double &roll);
  const Pose3d delta_odom_;
  const Pose3d delta_laser_;
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
  int size_{0};

  double extrinsic_x;
  double extrinsic_y;
  double extrinsic_z;
  double extrinsic_roll;
  double extrinsic_pitch;
  double extrinsic_yaw;
  Eigen::Matrix4d extrinsic_matrix;


  void reset();

  bool calib_tag{true};
};
