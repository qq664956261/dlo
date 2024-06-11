#include "hj/ExtrinsicErrorTerm.hh"
#include<pcl/common/transforms.h>
ExtrinsicErrorTerm::ExtrinsicErrorTerm()
{
}

ExtrinsicErrorTerm::~ExtrinsicErrorTerm()
{
}



ceres::vn_fusion::Pose3d CalculatePoseMean(const std::vector<ceres::vn_fusion::Pose3d>& poses)
{
    ceres::vn_fusion::Pose3d mean_pose = poses[0];
    for (size_t i = 1; i < poses.size(); i++)
    {
        auto last_one = mean_pose;
        mean_pose.p = (double)i / (i + 1) * last_one.p + 1.0 / (i + 1) * poses[i].p;
        mean_pose.q = last_one.q.slerp(1.0 / (i + 1), poses[i].q);
    }
    return mean_pose;
}

int ExtrinsicErrorTerm::runCalibExtrinsicRansac(VecOfPoses odom_poses, VecOfPoses laser_poses, double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
{
    if (!calib_tag)
    {
        std::cout << u8"请使用标定模式数据" << std::endl;
        return MPEMsg::Tag_Err;
    }
    size_ = laser_poses.size();
    cout << "odom size=" << odom_poses.size() << ";laser size=" << laser_poses.size() << ";ori size=" << size_ << endl;
    if (size_ < 60) {
        std::cout << u8"标定数据异常: 关键帧数量过少" << std::endl;
        return MPEMsg::LessKFs_Err;
    }
    if ((size_ - laser_poses.size()) > (0.35 * size_))
    {
        std::cout << u8"标定失败: 发现里程计与激光的间隔存在异常差距" << std::endl;
        return MPEMsg::LargeKFGap_Err;
    }
    VecOfPoses odom_delta_poses;
    VecOfPoses laser_delta_poses;
    for (int i = 0; i + 1 < (int)odom_poses.size() && i + 1 < (int)laser_poses.size(); i++)
    {
        odom_delta_poses.push_back(Between(odom_poses[i], odom_poses[i + 1]));
        laser_delta_poses.push_back(Between(laser_poses[i], laser_poses[i + 1]));
    }
    cout << "odom delta size=" << odom_delta_poses.size() << ";laser delta size=" << laser_delta_poses.size() << endl;
    cout << "before opt:"
        << "\n extrinsic_x: " << x
        << "\n extrinsic_y: " << y
        << "\n extrinsic_z: " << z
        << "\n extrinsic_yaw: " << yaw
        << "\n extrinsic_pitch: " << pitch
        << "\n extrinsic_roll: " << roll
        << endl;


    int prune_size = 3;
    vector<vector<int>> prune_head_indices(0);

    int head_i = 10;
    int head_j = 23;
    while (head_j + 13 < odom_delta_poses.size())
    {
        vector<int> head_ij{ head_i, head_j };
        prune_head_indices.push_back(head_ij);

        cout << "head_ij:" << head_i << "," << head_j << endl;
        if (head_j + 13 < odom_delta_poses.size())
        {
            head_j++;
        }
        else
        {
            head_i++;
            head_j = head_i + 13;
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 6;
    options.max_solver_time_in_seconds = 600;
    options.max_num_iterations = 1000;

    Eigen::Matrix<double, 6, 6> sqrt_info;
    sqrt_info.setZero();
    sqrt_info(0, 0) = 1. / 0.05;
    sqrt_info(1, 1) = 1. / 0.02;
    sqrt_info(2, 2) = 1. / 0.02;
    sqrt_info(3, 3) = sqrt_info(4, 4) = sqrt_info(5, 5) = 1. / (2. * M_PI / 180.);

    vector<ceres::vn_fusion::Pose3d> results;
    for (auto&& head_ij : prune_head_indices)
    {
        Pose3d param{ Eigen::Vector3d(x, y, z), ypr2Quat(yaw, pitch, roll) };
        //Eigen::Quaterniond param_q(ypr2R(Eigen::Vector3d(yaw, pitch, roll)));
        //Pose3d param{ Eigen::Vector3d(x, y, z),param_q };

        std::cout << "param.q.toRotationMatrix()1:" << param.q.toRotationMatrix() << std::endl;
        ceres::LocalParameterization* quat_local_param = new ceres::EigenQuaternionParameterization;
        ceres::Problem problem;
        for (size_t i = 0; i < odom_delta_poses.size() && i < laser_delta_poses.size(); i++)
        {
            if ((i >= head_ij[0] && i < head_ij[0] + 3) || (i >= head_ij[1] && i < head_ij[1] + 3))
                continue;
            ceres::CostFunction* cost_function = ExtrinsicErrorTerm::Create(
                odom_delta_poses[i], laser_delta_poses[i], sqrt_info);
            problem.AddResidualBlock(cost_function, NULL, param.p.data(), param.q.coeffs().data());
        }
        problem.SetParameterization(param.q.coeffs().data(), quat_local_param);

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        results.push_back(param);
    }
    cout << "results.size=" << results.size() << endl;

    auto mean_pose = CalculatePoseMean(results);
    vector<ceres::vn_fusion::Pose3d> pose_mean_errs;
    vector<double> dist_mean_errs;
    vector<double> angle_mean_errs;
    for (auto&& result : results)
    {
        ceres::vn_fusion::Pose3d err;
        err.p = result.p - mean_pose.p;
        err.q = mean_pose.q.inverse() * result.q;
        dist_mean_errs.push_back(err.p.norm());
        angle_mean_errs.push_back(fabs(Eigen::AngleAxisd(err.q).angle()));
    }

    // ?????????????10%
    auto dist_errs_copy = dist_mean_errs;
    auto angle_errs_copy = angle_mean_errs;
    std::sort(dist_mean_errs.begin(), dist_mean_errs.end());
    std::sort(angle_mean_errs.begin(), angle_mean_errs.end());
    size_t size_to_quit = results.size() / 10;
    vector<ceres::vn_fusion::Pose3d> remain_poses;
    for (size_t i = 0, isize = results.size(); i < isize; i++)
    {
        if (dist_errs_copy.at(i) <= dist_mean_errs.at(isize - size_to_quit) &&
            angle_errs_copy.at(i) <= angle_mean_errs.at(isize - size_to_quit))
        {
            remain_poses.push_back(results.at(i));
        }
    }

    //cout << summary.BriefReport() << endl;
    cout << "remain.size=" << remain_poses.size() << endl;
    auto param = CalculatePoseMean(remain_poses);
    auto ypr = R2ypr(param.q.toRotationMatrix());
 

    extrinsic_x = param.p[0];
    extrinsic_y = param.p[1];
    extrinsic_z = param.p[2];
    extrinsic_yaw = ypr[0];
    extrinsic_pitch = ypr[1];
    extrinsic_roll = ypr[2];
    cout << "after opt:"
        << "\n extrinsic_x: " << param.p[0]
        << "\n extrinsic_y: " << param.p[1]
        << "\n extrinsic_z: " << param.p[2]
        << "\n extrinsic_yaw: " << ypr[0]
        << "\n extrinsic_pitch: " << ypr[1]
        << "\n extrinsic_roll: " << ypr[2]
        << endl;
    x = extrinsic_x;
    y = extrinsic_y;
    z = extrinsic_z;
    yaw = extrinsic_yaw;
    pitch = extrinsic_pitch;
    roll = extrinsic_roll;
    return MPEMsg::CalibExtrinsicSuccess;
}




int ExtrinsicErrorTerm::runCalibExtrinsic(VecOfPoses odom_poses, VecOfPoses laser_poses, double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
{

    cout << "odom size=" << odom_poses.size() << ";laser size=" << laser_poses.size() << ";ori size=" << size_ << endl;
    if (size_ < 60) {
        std::cout << u8"标定数据异常: 关键帧数量过少" << std::endl;
        return MPEMsg::LessKFs_Err;
    }
    if ((size_ - laser_poses.size()) > (0.35 * size_))
    {
        std::cout << u8"标定失败: 发现里程计与激光的间隔存在异常差距" << std::endl;
        return MPEMsg::LargeKFGap_Err;
    }
    VecOfPoses odom_delta_poses;
    VecOfPoses laser_delta_poses;
    for (int i = 0; i + 1 < (int)odom_poses.size() && i + 1 < (int)laser_poses.size(); i++)
    {
        odom_delta_poses.push_back(Between(odom_poses[i], odom_poses[i + 1]));
        laser_delta_poses.push_back(Between(laser_poses[i], laser_poses[i + 1]));
    }
    cout << "odom delta size=" << odom_delta_poses.size() << ";laser delta size=" << laser_delta_poses.size() << endl;
    /*
    cout << "before opt:"
        << "\n extrinsic_x: " << argv[2]
        << "\n extrinsic_y: " << argv[3]
        << "\n extrinsic_z: " << argv[4]
        << "\n extrinsic_yaw: " << argv[5]
        << "\n extrinsic_pitch: " << argv[6]
        << "\n extrinsic_roll: " << argv[7]
        << endl;
        */
    Pose3d param{ Eigen::Vector3d(x, y, z),
                 ypr2Quat(yaw, pitch, roll) };
    ;    std::cout << "param.q.toRotationMatrix()1:" << param.q.toRotationMatrix() << std::endl;
    ceres::Problem problem;
    ceres::LocalParameterization* quat_local_param = new ceres::EigenQuaternionParameterization;
    Eigen::Matrix<double, 6, 6> sqrt_info;
    sqrt_info.setZero();
    sqrt_info(0, 0) = 1. / 0.05;
    sqrt_info(1, 1) = 1. / 0.02;
    sqrt_info(2, 2) = 1. / 0.02;
    sqrt_info(3, 3) = sqrt_info(4, 4) = sqrt_info(5, 5) = 1. / (2. * M_PI / 180.);
    for (size_t i = 0; i < odom_delta_poses.size() && i < laser_delta_poses.size(); i++)
    {
        ceres::CostFunction* cost_function = ExtrinsicErrorTerm::Create(
            odom_delta_poses[i], laser_delta_poses[i], sqrt_info);
        problem.AddResidualBlock(cost_function, NULL, param.p.data(), param.q.coeffs().data());
    }
    problem.SetParameterization(param.q.coeffs().data(), quat_local_param);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 600;
    options.max_num_iterations = 1000;

    options.function_tolerance = 1e-20;
    options.gradient_tolerance = 1e-20;
    options.parameter_tolerance = 1e-20;

    /*
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.gradient_check_relative_precision = 1e-4;
    */
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
    auto ypr = getYpr(param.q.toRotationMatrix());
    x = param.p[0];
    y = param.p[1];
    z = param.p[2];
    yaw = ypr[0];
    pitch = ypr[1];
    roll = ypr[2];
    extrinsic_x = param.p[0];
    extrinsic_y = param.p[1];
    extrinsic_z = param.p[2];
    extrinsic_yaw = ypr[0];
    extrinsic_pitch = ypr[1];
    extrinsic_roll = ypr[2];
    cout << "after opt:"
        << "\n extrinsic_x: " << param.p[0]
        << "\n extrinsic_y: " << param.p[1]
        << "\n extrinsic_z: " << param.p[2]
        << "\n extrinsic_yaw: " << ypr[0]
        << "\n extrinsic_pitch: " << ypr[1]
        << "\n extrinsic_roll: " << ypr[2]
        << endl;
    std::cout << "param.q.toRotationMatrix()2:" << param.q.toRotationMatrix() << std::endl;

    /*
    Eigen::Quaterniond q = ypr2Quat(ypr[0], ypr[1], ypr[2]);
    Eigen::Matrix3d R = q.toRotationMatrix();
    std::cout << "R.matrix():" << R.matrix() << std::endl;
    Eigen::Matrix3d temp_R;
    temp_R << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    R = R * temp_R;
    Eigen::Quaterniond q1(R);
    Eigen::Matrix<double, 3, 1> ypr1 = getYpr(R);
    cout << "after opt:"
         << "\n extrinsic_x: " << param.p[0]
         << "\n extrinsic_y: " << param.p[1]
         << "\n extrinsic_z: " << param.p[2]
         << "\n extrinsic_yaw: " << ypr1[0]
         << "\n extrinsic_pitch: " << ypr1[1]
         << "\n extrinsic_roll: " << ypr1[2];
*/
   
    return MPEMsg::CalibExtrinsicSuccess;
}



void ExtrinsicErrorTerm::reset()
{
    size_ = 0;
}

