#include <hj/PointReader.hh>

MapPointReader::MapPointReader()
{
}

MapPointReader::~MapPointReader()
{
}

int MapPointReader::read_pts(const std::string filepath, std::vector<cv::Point3f> &pts)
{
  printf("Read mappoints file %s.\n", filepath.c_str());
  int ret = -1; // -1: 正常  0: 无法打开文件 1:读的数据位置不对
  pts.clear();

  //打开文件
  std::string map_pointsfile = filepath;
  printf("path:%s", map_pointsfile.c_str());
  std::ifstream readText;
  readText.open(map_pointsfile.c_str()); // read
  if (!readText.is_open())
  {
    printf("Error in MPR.read_pts()! Can not open points file %s!\n", filepath.c_str());
    ret = 0;
    return ret;
  }

  char temp[300];
  int count = 0;
  while (!readText.eof())
  {
    readText.getline(temp, 300);
    count++;
    if (count >= 1000000)
      break;
    if (count <= 1)
      continue; //从第二行开始

    float pt_x, pt_y, pt_z;
    sscanf(temp, "%f %f %f", &pt_x, &pt_y, &pt_z);
    pts.push_back(cv::Point3f(pt_x, pt_y, pt_z));
  }

  readText.close();
  int psize = pts.size();
  printf("RP: readpoints num  =  %d.\n", psize);

  // test check
  // printf("Check: point[0]: %f, %f, %f. ",pts[0].x,pts[0].y,pts[0].z);
  // printf("point[100]: %f, %f, %f. \n",pts[100].x,pts[100].y,pts[100].z);
  // printf("point[%d]: %f, %f, %f. ",(psize-2),pts[psize-2].x,pts[psize-2].y,pts[psize-2].z);
  // printf("point[%d]: %f, %f, %f. \n",(psize-1),pts[psize-1].x,pts[psize-1].y,pts[psize-1].z);
  return ret;
}

int MapPointReader::save_pts(const std::string filepath)
{
  int ret = -1;
  _readfilepath = filepath;

  return ret;
}

int MapPointReader::saveRec(const std::vector<MyRectangle> rectangles, const std::string savepath)
{
  int ret = -1;
  if (rectangles.size() == 0)
  {
    printf("NONE for rectangles saving.\n");
    return ret;
  }

  FILE *fp = NULL;
  fp = fopen(savepath.c_str(), "a+");
  if (fp == NULL)
  {
    printf("-----------------------It fails to creat a new .txt file for rectangle data in %s.------------------\n\n", savepath.c_str());
    ret = 0;
    return ret;
  }
  fclose(fp);
  // printf("Rectangle file is created.");

  float lx, ly, lz, rx, ry, rz;
  int rsize = rectangles.size();
  std::ofstream openfile;
  openfile.open(savepath, std::ios::app);
  for (int i = 0; i < rsize; i++)
  {

    openfile << i << " ";
    openfile << rectangles[i].topLeft.x << " " << rectangles[i].topLeft.y << " " << rectangles[i].topLeft.z << " ";
    openfile << rectangles[i].bottomRight.x << " " << rectangles[i].bottomRight.y << " " << rectangles[i].bottomRight.z << " ";
    openfile << std::endl;
  }
  openfile.close();
  printf("Rectangle file is created.");

  return ret;
}

int MapPointReader::saveAll(const std::vector<cv::Point3f> _pts, const std::string savepath)
{
  int ret = -1;

  return ret;
}

int MapPointReader::read_pose_tum(const std::string filepath, std::vector<pose_tum> &poses)
{
  printf("Read pose_tum file %s\n", filepath.c_str());
  int ret = -1; // -1: 正常  0: 无法打开文件 1:读的数据位置不对
  poses.clear();
  std::ifstream readText;
  readText.open(filepath.c_str());
  if (!readText.is_open())
  {
    printf("Error in MPR.read_pose_tum()! %s\n", filepath.c_str());
    ret = 0;
    return ret;
  }

  char temp[300];
  while (!readText.eof())
  {
    pose_tum temp_pose;

    readText.getline(temp, 300);
    float tx, ty, tz, qx, qy, qz, qw;
    sscanf(temp, "%lf %f %f %f %f %f %f %f",
           &(temp_pose.timestamp), &tx, &ty, &tz, &qx, &qy, &qz, &qw);
    temp_pose.t_xyz = cv::Point3f(tx, ty, tz);
    temp_pose.q_xyzw = Eigen::Quaterniond(qw, qx, qy, qz);

    poses.push_back(temp_pose);
  }

  readText.close();
  printf("poses size(): %zu. \n", poses.size());
  return ret;
}

int MapPointReader::read_pose_pulse(const std::string filepath, std::vector<pulse_tum> &pulses)
{
  printf("read_pose_pulse() %s.\n", filepath.c_str());
  int ret = -1;
  pulses.clear();

  std::string pulse_file = filepath;
  std::ifstream readText;
  readText.open(pulse_file.c_str());
  if (!readText.is_open())
  {
    printf("Error in MPR.read_pose_pulse()! %s\n", filepath.c_str());
    ret = 0;
    return ret;
  }

  char temp[300];
  int count = 0;
  while (!readText.eof())
  {
    readText.getline(temp, 300); // read evofile
    count++;
    if (count < 2) //从第二行开始
      continue;
    double time1, time2;
    float ori_x, avg_x, x_diff, ori_l, avg_l, l_diff, diff_l_2D, diff_l_3D;
    sscanf(temp, "%lf %lf %f %f %f %f %f %f %f %f",
           &time1, &time2, &ori_x, &avg_x, &x_diff, &ori_l, &avg_l, &l_diff, &diff_l_2D, &diff_l_3D);

    pulse_tum temp_pulse;
    temp_pulse.timestamp1 = time1;
    temp_pulse.timestamp2 = time2;
    temp_pulse.x_diff = x_diff;       //这是x
    temp_pulse.diff_l_2D = diff_l_2D; //这是 diff_l_2D
    temp_pulse.diff_l_3D = diff_l_3D; //这是 diff_l_3D
    temp_pulse.show_diff = diff_l_2D; //用于显示
    // printf("temp_pulse, time1: %lf, time2: %lf, x_diff: %f.\n",time1,time2,x_diff);
    pulses.push_back(temp_pulse);
  }

  readText.close();

  // check
  //  for (int i =0;i<pulses.size();i++){
  //      printf("pulses[%d], time1: %lf, time2: %lf, x_diff: %f.\n",i,pulses[i].timestamp1,pulses[i].timestamp2,pulses[i].x_diff);
  //  }

  return ret;
}

// int MapPointReader::read_kPose(const std::string filepath, std::vector<kPose> &kPoses){
//     printf("read_kPose() %s.\n", filepath.c_str());
//     int ret = -1;
//     kPoses.clear();
//
//     std::string pulse_file = filepath;
//     std::ifstream readText;
//     readText.open(pulse_file.c_str());
//     if (!readText.is_open()){
//         printf("Error in MPR.read_pose_pulse()! %s\n", filepath.c_str());
//         ret = 0;
//         return ret;
//     }
//
//     char temp[300];
//     int count = 0;
//     while(readText.peek() != EOF){
//         readText.getline(temp,300);//read evofile
//         count ++;
//         if (count<2)//从第二行开始
//             continue;
//         int kfId;
//         float x,y,z,intensity,roll,pitch,yaw,time;
//         sscanf(temp, "%d %f %f %f %f %f %f %f %f",
//                &kfId, &x, &y, &z, &intensity, &roll, &pitch, &yaw, &time);
//
//         kPose temp_pulse;
//         temp_pulse.keyFrameId = kfId;
//         Eigen::Vector3f translation;
//         translation<< x , y , z;
//         temp_pulse.t = translation;
//         Eigen::Vector3f ea(roll ,pitch, yaw);
//         Eigen::Matrix3f rotation;
//         Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(ea(0), Eigen::Vector3f::UnitX()));
//         Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(ea(1), Eigen::Vector3f::UnitY()));
//         Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(ea(2), Eigen::Vector3f::UnitZ()));
//         rotation = rollAngle * pitchAngle * yawAngle;
//         temp_pulse.R = rotation;
//         Eigen::Vector3f  ea11= temp_pulse.R.eulerAngles(2,1,0);
//
//         std::cout<<"temp_pulse.R.matrix():"<<ea11 <<std::endl;
//         printf("temp_pulse, kfId:%d .\n",kfId);
//         kPoses.push_back(temp_pulse);
//     }
//
//     readText.close();
//
//     return ret;
// }
//
// int MapPointReader::read_HiPoint(const std::string filepath, std::vector<HiPoint> &HiPoints){
//     printf("read_pose_pulse() %s.\n", filepath.c_str());
//     int ret = -1;
//     HiPoints.clear();
//
//     std::string pulse_file = filepath;
//     std::ifstream readText;
//     readText.open(pulse_file.c_str());
//     if (!readText.is_open()){
//         printf("Error in MPR.read_pose_pulse()! %s\n", filepath.c_str());
//         ret = 0;
//         return ret;
//     }
//
//     char temp[300];
//     int count = 0;
//     while(readText.peek() != EOF){
//         readText.getline(temp,300);//read evofile
//         count ++;
//         if (count<2)//从第二行开始
//             continue;
//         int kfId;
//         float pointId,x,y,z;
//         sscanf(temp, "%d %f %f %f %f",
//                &kfId, &pointId, &x, &y, &z);
//
//         HiPoint temp_pulse;
//         temp_pulse.keyFrameId = kfId;
//         Eigen::Vector3f point;
//         point<< x , y , z;
//         temp_pulse.point = point;
//
//         printf("temp_pulse, x: %f, y: %f, z: %f.\n",x,y,z);
//         HiPoints.push_back(temp_pulse);
//     }
//
//     readText.close();
//
//     return ret;
// }

int MapPointReader::read_HiPoint(const std::string filepath, std::vector<HiPoint, Eigen::aligned_allocator<HiPoint>> &HiPoints)
{
  printf("read_pose_pulse() %s.\n", filepath.c_str());
  int ret = -1;
  HiPoints.clear();
  std::string pulse_file = filepath;
  std::vector<std::string> filenames;
  getFiles(pulse_file, filenames);
  printf("filenames.size(): %d.\n", filenames.size());
  for (int i = 0; i < filenames.size(); i++)
  {
    std::string HiPointPath = filenames[i] + "/cloud_HIs.pcd";
    pcl::PointCloud<pcl::PointDEM>::Ptr cloud(new pcl::PointCloud<pcl::PointDEM>);
    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointDEM>(HiPointPath, *cloud) == -1)
    {
      PCL_ERROR("Couldn't read file pcd\n");
      continue;
    }
    for (int j = 0; j < cloud->points.size(); j++)
    {
      HiPoint temp_pulse;
      temp_pulse.keyFrameId = i;
      Eigen::Vector3d point;
      point << cloud->points[j].x, cloud->points[j].y, cloud->points[j].z;
      temp_pulse.point = point;
      temp_pulse.intensity = cloud->points[j].intensity;
      temp_pulse.intensity_variance = cloud->points[j].intensity_variance;
      temp_pulse.height_variance = cloud->points[j].height_variance;
      HiPoints.push_back(temp_pulse);
      // printf("read_HiPoint, x: %f, y: %f, z: %f , id:%d.\n",temp_pulse.point.x(),temp_pulse.point.y(),temp_pulse.point.z(),temp_pulse.keyFrameId);
    }
  }
  return ret;
}

int MapPointReader::read_cornerPoint(const std::string filepath, std::vector<cornerPoint, Eigen::aligned_allocator<cornerPoint>> &cornerPoints)
{
  printf("read_pose_pulse() %s.\n", filepath.c_str());
  int ret = -1;
  cornerPoints.clear();
  std::string pulse_file = filepath;
  std::vector<std::string> filenames;
  getFiles(pulse_file, filenames);
  printf("filenames.size(): %d.\n", filenames.size());
  for (int i = 0; i < filenames.size(); i++)
  {
    std::string HiPointPath = filenames[i] + "/cloud_corners.pcd";
    pcl::PointCloud<pcl::PointDEM>::Ptr cloud(new pcl::PointCloud<pcl::PointDEM>);
    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointDEM>(HiPointPath, *cloud) == -1)
    {
      PCL_ERROR("Couldn't read file pcd\n");
      continue;
    }
    for (int j = 0; j < cloud->points.size(); j++)
    {
      cornerPoint temp_pulse;
      temp_pulse.keyFrameId = i;
      Eigen::Vector3d point;
      point << cloud->points[j].x, cloud->points[j].y, cloud->points[j].z;
      temp_pulse.point = point;
      temp_pulse.intensity = cloud->points[j].intensity;
      temp_pulse.intensity_variance = cloud->points[j].intensity_variance;
      temp_pulse.height_variance = cloud->points[j].height_variance;
      cornerPoints.push_back(temp_pulse);
      // printf("read_cornerPoint, x: %f, y: %f, z: %f , id:%d.\n",temp_pulse.point.x(),temp_pulse.point.y(),temp_pulse.point.z(),temp_pulse.keyFrameId);
    }
  }
  return ret;
}
int MapPointReader::read_surPoint(const std::string filepath, std::vector<surPoint, Eigen::aligned_allocator<surPoint>> &surPoints)
{
  printf("read_pose_pulse() %s.\n", filepath.c_str());
  int ret = -1;
  surPoints.clear();
  std::string pulse_file = filepath;
  std::vector<std::string> filenames;
  getFiles(pulse_file, filenames);
  printf("filenames.size(): %d.\n", filenames.size());
  for (int i = 0; i < filenames.size(); i++)
  {
    std::string HiPointPath = filenames[i] + "/cloud_surfs.pcd";
    pcl::PointCloud<pcl::PointDEM>::Ptr cloud(new pcl::PointCloud<pcl::PointDEM>);
    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointDEM>(HiPointPath, *cloud) == -1)
    {
      PCL_ERROR("Couldn't read file pcd\n");
      continue;
    }
    for (int j = 0; j < cloud->points.size(); j++)
    {
      surPoint temp_pulse;
      temp_pulse.keyFrameId = i;
      Eigen::Vector3d point;
      point << cloud->points[j].x, cloud->points[j].y, cloud->points[j].z;
      temp_pulse.point = point;
      temp_pulse.intensity = cloud->points[j].intensity;
      temp_pulse.intensity_variance = cloud->points[j].intensity_variance;
      temp_pulse.height_variance = cloud->points[j].height_variance;
      surPoints.push_back(temp_pulse);
      // printf("read_surPoint, x: %f, y: %f, z: %f , id:%d.\n",temp_pulse.point.x(),temp_pulse.point.y(),temp_pulse.point.z(),temp_pulse.keyFrameId);
    }
  }
  return ret;
}
int MapPointReader::read_outlierPoint(const std::string filepath, std::vector<outlierPoint, Eigen::aligned_allocator<outlierPoint>> &outlierPoints)
{
  printf("read_pose_pulse() %s.\n", filepath.c_str());
  int ret = -1;
  outlierPoints.clear();
  std::string pulse_file = filepath;
  std::vector<std::string> filenames;
  getFiles(pulse_file, filenames);
  printf("filenames.size(): %d.\n", filenames.size());
  for (int i = 0; i < filenames.size(); i++)
  {
    std::string outlierPointPath = filenames[i] + "/cloud_outliers.pcd";
    pcl::PointCloud<pcl::PointDEM>::Ptr cloud(new pcl::PointCloud<pcl::PointDEM>);
    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointDEM>(outlierPointPath, *cloud) == -1)
    {
      PCL_ERROR("Couldn't read file pcd\n");
      continue;
    }
    for (int j = 0; j < cloud->points.size(); j++)
    {
      outlierPoint temp_pulse;
      temp_pulse.keyFrameId = i;
      Eigen::Vector3d point;
      point << cloud->points[j].x, cloud->points[j].y, cloud->points[j].z;
      temp_pulse.point = point;
      temp_pulse.intensity = cloud->points[j].intensity;
      temp_pulse.intensity_variance = cloud->points[j].intensity_variance;
      temp_pulse.height_variance = cloud->points[j].height_variance;
      outlierPoints.push_back(temp_pulse);
      // printf("read_surPoint, x: %f, y: %f, z: %f , id:%d.\n",temp_pulse.point.x(),temp_pulse.point.y(),temp_pulse.point.z(),temp_pulse.keyFrameId);
    }
  }
  return ret;
}
void MapPointReader::getFiles(const std::string &rootPath, std::vector<std::string> &ret)
{
  boost::filesystem::path fullpath(rootPath);
  boost::filesystem::recursive_directory_iterator end_iter;
  int i = 0;
  while (1)
  {
    std::string snum;
    if (i < 10)
    {
      boost::format fmt("%s%d");
      snum = (fmt % "00000" % i).str();
    }
    if (i >= 10 && i < 100)
    {
      boost::format fmt("%s%d");
      snum = (fmt % "0000" % i).str();
    }
    if (i >= 100 && i < 1000)
    {
      boost::format fmt("%s%d");
      snum = (fmt % "000" % i).str();
    }
    if (i >= 1000 && i < 10000)
    {
      boost::format fmt("%s%d");
      snum = (fmt % "00" % i).str();
    }
    std::string file_name = rootPath + snum;
    // printf("getFiles:%s.\n",file_name.c_str());
    // printf("getFiles:%s.\n",snum.c_str());
    if (boost::filesystem::is_directory(file_name))
    {
      // std::cout << "is dir" << std::endl;
      ret.push_back(file_name);
      i++;
    }
    else
    {
      break;
    }
  }
}
