#ifndef MAPEDITOR_POINTREADER_HH
#define MAPEDITOR_POINTREADER_HH

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>
#include <opencv2/core.hpp>
enum class optimizedMapState :int
{
    uninit = 0,
    wallConstraint1 = 1,
    wallConstraint2 = 2,
    wallConstraint3 = 3,
    wallConstraint4 = 4,
    wallConstraint5 = 5,
    wallConstraint6 = 6,
    wallConstraint7 = 7,
    wallConstraint8 = 8,
    wallParallelConstraint1 = 9,
    wallParallelConstraint2 = 10,
    wallParallelConstraint3 = 11,
    wallParallelConstraint4 = 12,
    wallParallelConstraint5 = 13,
    wallParallelConstraint6 = 14,
    wallParallelConstraint7 = 15,
    wallParallelConstraint8 = 16,
    wallThicknessConstraint1 = 17,
    wallThicknessConstraint2 = 18,
    wallThicknessConstraint3 = 19,
    wallThicknessConstraint4 = 20,
    wallThicknessConstraint5 = 21,
    wallThicknessConstraint6 = 22,
    wallThicknessConstraint7 = 23,
    wallThicknessConstraint8 = 24,
    walGroundConstraint1 = 25,
    walGroundConstraint2 = 26,
    walGroundConstraint3 = 27,
    walGroundConstraint4 = 28,
    walGroundConstraint5 = 29,
    walGroundConstraint6 = 30,
    walGroundConstraint7 = 31,
    walGroundConstraint8 = 32


};
enum class LocQulity :int
{
    GOOD = 0,
    AVER = 1,
    BAD = 2,
    UNINIT = 3
};
struct jumpInfo
{
    float x;
    float y;
    float z;
    float dx;
    float dy;
    float dz;
    float drift_x;
    float drift_y;
    float drift_fabs_x;
    float drift_fabs_y;

};
struct HiMarkInfo
{
    float x_world;
    float y_world;
    float x_lidar;
    float y_lidar;
    float radius;
};

struct wallInfo
{
    std::vector<cv::Point3f> vBenchmarkPoints;
    std::vector<cv::Point3f> vConstraintedPoints;
    std::vector<cv::Point3f> vKeyFramePoints;
    bool x;
    bool y;
};
struct rotateWallInfo
{
    std::vector<cv::Point3f> vBenchmarkPoints;
    bool x;
    bool y;
};
struct wallCornerInfo
{
    std::vector<cv::Point3f> vPoints;
    float topLeft_x;
    float topLeft_y;
    float bottomLeft_x;
    float bottomLeft_y;
    float topRight_x;
    float topRight_y;
    float bottomRight_x;
    float bottomRight_y;
    float x_picked;
    float y_picked;
    float wallThickness_x;
    float wallThickness_y;
};
struct planeInfo
{
    std::vector<cv::Point3f> vPoints;
};

struct MyRectangle
{
    cv::Point3f topLeft;
    cv::Point3f bottomRight;
};

struct pose_tum
{
    double timestamp;
    cv::Point3f t_xyz;
    Eigen::Quaterniond q_xyzw;
};

struct pulse_tum
{
    double timestamp1;
    double timestamp2;
    float show_diff; //用于显示
    float x_diff;
    float diff_l_2D;
    float diff_l_3D;
};

struct HiPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;
    int keyFrameId;
    float intensity;
    float intensity_variance;
    double height_variance;
};
struct cornerPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;
    int keyFrameId;
    float intensity;
    float intensity_variance;
    double height_variance;
};
struct surPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;
    int keyFrameId;
    float intensity;
    float intensity_variance;
    double height_variance;
};
struct outlierPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;
    int keyFrameId;
    float intensity;
    float intensity_variance;
    double height_variance;
};
class MapPointReader
{
public:
    MapPointReader();
    ~MapPointReader();

    int read_pts(const std::string filepath, std::vector<cv::Point3f> &pts);
    int save_pts(const std::string filepath);

    int saveRec(const std::vector<MyRectangle> rectangles, const std::string savepath); //存储长方形
    int saveAll(const std::vector<cv::Point3f> _pts, const std::string savepath);       //存储移除特定区域后的所有点云。

    // pose
    int read_pose_tum(const std::string filepath, std::vector<pose_tum> &poses);                                     //加载pose的位姿
    int read_pose_pulse(const std::string filepath, std::vector<pulse_tum> &pulses);                                 //加载pose的pulse
    int read_HiPoint(const std::string filepath, std::vector<HiPoint, Eigen::aligned_allocator<HiPoint>> &HiPoints); //加载带KfId的高反点
    int read_cornerPoint(const std::string filepath, std::vector<cornerPoint, Eigen::aligned_allocator<cornerPoint>> &cornerPoints);
    int read_surPoint(const std::string filepath, std::vector<surPoint, Eigen::aligned_allocator<surPoint>> &surPoints);
    int read_outlierPoint(const std::string filepath, std::vector<outlierPoint, Eigen::aligned_allocator<outlierPoint>> &outlierPoints);
    void getFiles(const std::string &rootPath, std::vector<std::string> &ret);

private:
    std::string _readfilepath;
    // void inter_readpts(const std::string filepath);
};

#endif // MAPEDITOR_POINTREADER_HH
