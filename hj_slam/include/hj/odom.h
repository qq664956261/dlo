/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "hj/dlo.h"

class hj::OdomNode
{

public:
  OdomNode(ros::NodeHandle node_handle);
  ~OdomNode();

  static void abort()
  {
    abort_ = true;
  }

  void start();
  void stop();

private:
  void abortTimerCB(const ros::TimerEvent &e);
  // void icpCB(const livox_ros_driver2::CustomMsgConstPtr& pc);
  void icpCB(const sensor_msgs::MultiEchoLaserScan::ConstPtr& multi_echo_msg);
  //void icpCB(const sensor_msgs::LaserScan::ConstPtr &pc);

  void imuCB(const sensor_msgs::Imu::ConstPtr &imu);
  //void imuCB(const hj_interface::ImuConstPtr &imu);
  bool saveTrajectory(hj_slam::save_traj::Request &req,
                      hj_slam::save_traj::Response &res);

  void getParams();

  void publishToROS();
  void publishPose();
  void publishTransform();
  void publishKeyframe();

  void preprocessPoints();
  void initializeInputTarget();
  void setInputSources();

  void initializeDLO();
  void gravityAlign();

  void getNextPose();
  void integrateIMU();

  void propagateS2S(Eigen::Matrix4f T);
  void propagateS2M();

  void setAdaptiveParams();

  void computeMetrics();
  void computeSpaciousness();

  void transformCurrentScan();
  void updateKeyframes();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void getSubmapKeyframes();
  void CallbackOdom(const hj_interface::PoseConstPtr &msg);
  void checkForTimeout(const ros::TimerEvent &);
  void runCalib();
  void buildCalibData();
  int timeStampSynchronization(double lidar_time);
  void saveResult(Eigen::Matrix4d T_odom_lidar);
  void align();
  void runLoopClosure();
  void LoopClosure(const int index1, const int index2, const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
  void debug();

  double first_imu_time;

  ros::NodeHandle nh;
  ros::Timer abort_timer;

  ros::ServiceServer save_traj_srv;

  ros::Subscriber icp_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber odom_sub;

  ros::Publisher odom_pub;
  ros::Publisher pose_pub;
  ros::Publisher keyframe_pub;
  ros::Publisher kf_pub;
  ros::Publisher curr_scan_pub;

  Eigen::Vector3f origin;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::Ptr>> keyframes;
  std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

  std::atomic<bool> dlo_initialized;
  std::atomic<bool> imu_calibrated;

  std::string odom_frame;
  std::string child_frame;

  pcl::PointCloud<PointType>::Ptr original_scan;
  pcl::PointCloud<PointType>::Ptr current_scan;
  pcl::PointCloud<PointType>::Ptr current_scan_t;

  pcl::PointCloud<PointType>::Ptr keyframes_cloud;
  pcl::PointCloud<PointType>::Ptr keyframe_cloud;
  int num_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  pcl::PointCloud<PointType>::Ptr submap_cloud;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;

  std::vector<int> submap_kf_idx_curr;
  std::vector<int> submap_kf_idx_prev;
  std::atomic<bool> submap_hasChanged;

  pcl::PointCloud<PointType>::Ptr source_cloud;
  pcl::PointCloud<PointType>::Ptr target_cloud;

  ros::Time scan_stamp;

  double curr_frame_stamp;
  double prev_frame_stamp;
  std::vector<double> comp_times;

  nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
  nano_gicp::NanoGICP<PointType, PointType> gicp;

  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> vf_scan;
  pcl::VoxelGrid<PointType> vf_submap;

  nav_msgs::Odometry odom;
  nav_msgs::Odometry kf;

  geometry_msgs::PoseStamped pose_ros;

  Eigen::Matrix4f T;
  Eigen::Matrix4f T_s2s, T_s2s_prev;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f pose_s2s;
  Eigen::Matrix3f rotSO3_s2s;
  Eigen::Quaternionf rotq_s2s;

  Eigen::Vector3f pose;
  Eigen::Matrix3f rotSO3;
  Eigen::Quaternionf rotq;

  Eigen::Matrix4f imu_SE3;

  struct XYZd
  {
    double x;
    double y;
    double z;
  };

  struct ImuBias
  {
    XYZd gyro;
    XYZd accel;
  };

  ImuBias imu_bias;

  struct ImuMeas
  {
    double stamp;
    XYZd ang_vel;
    XYZd lin_accel;
  };

  ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2)
  {
    return (m1.stamp < m2.stamp);
  };

  struct Metrics
  {
    std::vector<float> spaciousness;
  };

  Metrics metrics;

  static std::atomic<bool> abort_;
  std::atomic<bool> stop_publish_thread;
  std::atomic<bool> stop_publish_keyframe_thread;
  std::atomic<bool> stop_metrics_thread;
  std::atomic<bool> stop_debug_thread;

  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  std::mutex mtx_imu;
  std::mutex mtx_odom;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // Parameters
  std::string version_;

  bool gravity_align_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool initial_pose_use_;
  Eigen::Vector3f initial_position_;
  Eigen::Quaternionf initial_orientation_;

  bool crop_use_;
  double crop_size_;

  bool vf_scan_use_;
  double vf_scan_res_;

  bool vf_submap_use_;
  double vf_submap_res_;

  bool adaptive_params_use_;

  bool imu_use_;
  int imu_calib_time_;
  int imu_buffer_size_;

  int gicp_min_num_points_;

  int gicps2s_k_correspondences_;
  double gicps2s_max_corr_dist_;
  int gicps2s_max_iter_;
  double gicps2s_transformation_ep_;
  double gicps2s_euclidean_fitness_ep_;
  int gicps2s_ransac_iter_;
  double gicps2s_ransac_inlier_thresh_;

  int gicps2m_k_correspondences_;
  double gicps2m_max_corr_dist_;
  int gicps2m_max_iter_;
  double gicps2m_transformation_ep_;
  double gicps2m_euclidean_fitness_ep_;
  int gicps2m_ransac_iter_;
  double gicps2m_ransac_inlier_thresh_;

  bool first_frame_{false};
  bool use_first_frame_{false};
  std::vector<pcl::PointCloud<PointType>::Ptr> first_frame_clouds_;
  bool use_icp_{true};
  bool use_ndt_{false};
  pcl::IterativeClosestPoint<PointType, PointType> icp_s2s_;
  pcl::IterativeClosestPoint<PointType, PointType> icp_s2m_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_s2s_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_s2m_;
  hj_bf::HJSubscriber odom_sub_;
  ros::Timer timer_;
  ros::Time last_msg_time_;
  std::atomic<bool> start_calib_{false};
  std::deque<std::vector<double>> odom_data_;
  std::deque<std::vector<double>> lidar_data_;
  double lidar_time_;
  std::thread calib_thread_;
  std::atomic<bool> timeout_detected_ {false};
  ExtrinsicErrorTerm::Ptr error_term_;
  VecOfPoses odom_poses_;
  VecOfPoses lidar_poses_;
  std::deque<double> odom_times_;
  std::deque<double> lidar_times_;
  bool use_calib_{false};
  bool use_fuison_result_{false};
  std::vector<double> keyframes_timestamps_;
  double loop_distance_{0.5};
  std::atomic<bool> loop_detected_{false};
  double loop_time_thre_{30.0};
  double last_loop_time_{0.0};
  int loop_id_{0};
  double loop_yaw_{0.0};
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::Ptr>> keyframes_loop_;
  std::vector<double> keyframes_timestamps_loop_;
  pcl::PointCloud<PointType>::Ptr loop_source_cloud_;
  bool use_loop_{false};
  std::thread loop_thread_;
  std::mutex mtx_loop_;
  std::atomic<bool> is_optimized_{false};
  Eigen::Matrix4f T_before_;
  Eigen::Matrix4f T_after_;
  std::shared_ptr<SCManager> sc_manager_ptr_;
  int loop_size_{0};
};
