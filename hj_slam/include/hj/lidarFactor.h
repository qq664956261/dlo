
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

inline Eigen::Matrix3d skewd(const Eigen::Vector3d &x)
{
	Eigen::Matrix3d skew1 = Eigen::Matrix3d::Zero();
	skew1(0, 1) = -x[2];
	skew1(0, 2) = x[1];
	skew1(1, 0) = x[2];
	skew1(1, 2) = -x[0];
	skew1(2, 0) = -x[1];
	skew1(2, 1) = x[0];

	return skew1;
}

struct consecutivePose
{
	consecutivePose(Eigen::Vector4f q_constraint_, Eigen::Vector3d t_constraint_)
		: q_constraint(q_constraint_), t_constraint(t_constraint_) {}

	template <typename T>
	bool operator()(const T *q1, const T *t1, const T *q2, const T *t2, T *residual) const
	{
		Eigen::Quaternion<T> quaternion1{q1[3], q1[0], q1[1], q1[2]};
		Eigen::Matrix<T, 3, 1> translation1{t1[0], t1[1], t1[2]};
		Eigen::Quaternion<T> quaternion2{q2[3], q2[0], q2[1], q2[2]};
		Eigen::Matrix<T, 3, 1> translation2{t2[0], t2[1], t2[2]};
		Eigen::Quaternion<T> quaternion3{T(q_constraint[3]), T(q_constraint[0]), T(q_constraint[1]), T(q_constraint[2])};
		Eigen::Matrix<T, 3, 1> translation3{T(t_constraint[0]), T(t_constraint[1]), T(t_constraint[2])};
		Eigen::Quaternion<T> relative_q;
		Eigen::Matrix<T, 3, 1> relative_t;
		relative_q = quaternion2.inverse() * quaternion1;
		relative_t = quaternion2.inverse() * (translation1 - translation2);
		Eigen::Matrix<T, 3, 1> residual_q;
		residual_q = (relative_q * quaternion3.inverse()).vec();

		Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residual);
		residuals.template block<3, 1>(0, 0) = relative_t - translation3;
		residuals.template block<3, 1>(3, 0) = T(30) * residual_q;
		//Eigen::Matrix<T, 6, 6> sqrt_info = T(100) * Eigen::Matrix<T, 6, 6>::Identity();
		Eigen::Matrix<T, 6, 6> sqrt_info = T(100) * Eigen::Matrix<T, 6, 6>::Identity();

		residuals.applyOnTheLeft(sqrt_info);


		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector4f q_constraint_, const Eigen::Vector3d t_constraint_)
	{
		return (new ceres::AutoDiffCostFunction<
				consecutivePose, 6, 4, 3, 4, 3>(
			new consecutivePose(q_constraint_, t_constraint_)));
	}
	Eigen::Vector4f q_constraint;
	Eigen::Vector3d t_constraint;
};

struct loopPose
{
	loopPose(Eigen::Vector4f q_constraint_, Eigen::Vector3d t_constraint_)
		: q_constraint(q_constraint_), t_constraint(t_constraint_) {}

	template <typename T>
	bool operator()(const T *q1, const T *t1, const T *q2, const T *t2, T *residual) const
	{
		Eigen::Quaternion<T> quaternion1{q1[3], q1[0], q1[1], q1[2]};
		Eigen::Matrix<T, 3, 1> translation1{t1[0], t1[1], t1[2]};
		Eigen::Quaternion<T> quaternion2{q2[3], q2[0], q2[1], q2[2]};
		Eigen::Matrix<T, 3, 1> translation2{t2[0], t2[1], t2[2]};
		Eigen::Quaternion<T> quaternion3{T(q_constraint[3]), T(q_constraint[0]), T(q_constraint[1]), T(q_constraint[2])};
		Eigen::Matrix<T, 3, 1> translation3{T(t_constraint[0]), T(t_constraint[1]), T(t_constraint[2])};
		Eigen::Quaternion<T> relative_q;
		Eigen::Matrix<T, 3, 1> relative_t;
		relative_q = quaternion2.inverse() * quaternion1;
		relative_t = quaternion2.inverse() * (translation1 - translation2);
		Eigen::Matrix<T, 3, 1> residual_q;
		residual_q = (relative_q * quaternion3.inverse()).vec();

		Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residual);
		residuals.template block<3, 1>(0, 0) = relative_t - translation3;
		residuals.template block<3, 1>(3, 0) = T(30) * residual_q;
		//Eigen::Matrix<T, 6, 6> sqrt_info = T(100) * Eigen::Matrix<T, 6, 6>::Identity();
		Eigen::Matrix<T, 6, 6> sqrt_info = T(1000) * Eigen::Matrix<T, 6, 6>::Identity();

		residuals.applyOnTheLeft(sqrt_info);


		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector4f q_constraint_, const Eigen::Vector3d t_constraint_)
	{
		return (new ceres::AutoDiffCostFunction<
				loopPose, 6, 4, 3, 4, 3>(
			new loopPose(q_constraint_, t_constraint_)));
	}
	Eigen::Vector4f q_constraint;
	Eigen::Vector3d t_constraint;
};



