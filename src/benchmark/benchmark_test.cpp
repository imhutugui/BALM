#include "tools.hpp"
#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <ctime>
#include "bavoxel.hpp"
#include "sensor_json_parser.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <malloc.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace std;

geometry_msgs::Transform ToGeometryMsgTransform(const Eigen::Vector3d p, const Eigen::Quaterniond q)
{
    geometry_msgs::Transform tform;
    tform.translation.x = p.x();
    tform.translation.y = p.y();
    tform.translation.z = p.z();
    tform.rotation.w = q.w();
    tform.rotation.x = q.x();
    tform.rotation.y = q.y();
    tform.rotation.z = q.z();
    return tform;
}

std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromJson(
        const std::string& json_filename,
        tf2_ros::Buffer* const tf_buffer) {
    sensor_model cam0;
    sensor_model cam1;
    sensor_model cam2;
    sensor_model cam3;
    sensor_model lidar_horiz;
    sensor_model lidar_vert;
    sensor_model imu;
    parserIntrinsicJson(json_filename, cam0, cam1, cam2, cam3, lidar_horiz, lidar_vert, imu);

    std::vector<geometry_msgs::TransformStamped> transforms;

    // 添加imu的位姿信息
    geometry_msgs::TransformStamped imuTransform, baseTransform;
    imuTransform.transform =
            ToGeometryMsgTransform(
                Eigen::Vector3d(imu.position[0], imu.position[1], imu.position[2]),
            Eigen::Quaterniond(imu.orientation[3], imu.orientation[0],
            imu.orientation[1], imu.orientation[2]));

    imuTransform.child_frame_id = imu.name;
    imuTransform.header.frame_id = "base_link";
    tf_buffer->setTransform(imuTransform, "sensor_json", true);
    transforms.push_back(imuTransform);
    ROS_INFO("IMU frame: %s", imu.name.c_str());

    baseTransform.transform = ToGeometryMsgTransform(Eigen::Vector3d(0,0,0), Eigen::Quaterniond(1,0,0,0));
    baseTransform.child_frame_id = "base_link";
    baseTransform.header.frame_id = "trajectory_0";
    tf_buffer->setTransform(baseTransform, "sensor_json", true);
    transforms.push_back(baseTransform);

    // 添加水平lidar的位姿信息
    geometry_msgs::TransformStamped lidarHorizTransform;
    lidarHorizTransform.transform =
            ToGeometryMsgTransform(
                Eigen::Vector3d(lidar_horiz.position[0], lidar_horiz.position[1], lidar_horiz.position[2]),
            Eigen::Quaterniond(lidar_horiz.orientation[3], lidar_horiz.orientation[0],
            lidar_horiz.orientation[1], lidar_horiz.orientation[2]));

    lidarHorizTransform.child_frame_id = lidar_horiz.name;
    lidarHorizTransform.header.frame_id = imu.name;
    tf_buffer->setTransform(lidarHorizTransform, "sensor_json", true);
    transforms.push_back(lidarHorizTransform);
    ROS_INFO("laser horiz frame: %s", lidar_horiz.name.c_str());

    // 添加垂直lidar的位姿信息
    geometry_msgs::TransformStamped lidarVertTransfrom;
    lidarVertTransfrom.transform =
            ToGeometryMsgTransform(
                Eigen::Vector3d(lidar_vert.position[0], lidar_vert.position[1], lidar_vert.position[2]),
            Eigen::Quaterniond(lidar_vert.orientation[3], lidar_vert.orientation[0],
            lidar_vert.orientation[1], lidar_vert.orientation[2]));

    lidarVertTransfrom.child_frame_id = lidar_vert.name;
    lidarVertTransfrom.header.frame_id = lidar_horiz.name;
    tf_buffer->setTransform(lidarVertTransfrom, "sensor_json", true);
    transforms.push_back(lidarVertTransfrom);
    ROS_INFO("laser vert frame: %s", lidar_vert.name.c_str());

    return transforms;
}

template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "map";
  output.header.stamp = ros::Time::now();
  pub.publish(output);
}

ros::Publisher pub_path, pub_test, pub_show, pub_cute;

int read_pose(vector<double> &tims, PLM(3) &rots, PLV(3) &poss, string prename)
{
  string readname = prename + "alidarPose.csv";

  cout << readname << endl;
  ifstream inFile(readname);

  if(!inFile.is_open())
  {
    printf("open fail\n"); return 0;
  }

  int pose_size = 0;
  string lineStr, str;
  Eigen::Matrix4d aff;
  vector<double> nums;

  int ord = 0;
  while(getline(inFile, lineStr))
  {
    ord++;
    stringstream ss(lineStr);
    while(getline(ss, str, ','))
      nums.push_back(stod(str));

    if(ord == 4)
    {
      for(int j=0; j<16; j++)
        aff(j) = nums[j];

      Eigen::Matrix4d affT = aff.transpose();

      rots.push_back(affT.block<3, 3>(0, 0));
      poss.push_back(affT.block<3, 1>(0, 3));
      tims.push_back(affT(3, 3));
      nums.clear();
      ord = 0;
      pose_size++;
    }
  }

  return pose_size;
}

void read_file(vector<IMUST> &x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls, string &prename)
{
  prename = prename + "/datas/benchmark_realworld/";

  PLV(3) poss; PLM(3) rots;
  vector<double> tims;
  int pose_size = read_pose(tims, rots, poss, prename);
  
  for(int m=0; m<pose_size; m++)
  {
    string filename = prename + "full" + to_string(m) + ".pcd";

    pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
    pcl::PointCloud<pcl::PointXYZI> pl_tem;
    pcl::io::loadPCDFile(filename, pl_tem);
    for(pcl::PointXYZI &pp: pl_tem.points)
    {
      PointType ap;
      ap.x = pp.x; ap.y = pp.y; ap.z = pp.z;
      ap.intensity = pp.intensity;
      pl_ptr->push_back(ap);
    }

    pl_fulls.push_back(pl_ptr);

    IMUST curr;
    curr.R = rots[m]; curr.p = poss[m]; curr.t = tims[m];
    x_buf.push_back(curr);
  }
  

}

void data_show(vector<IMUST> x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls)
{
  IMUST es0 = x_buf[0];
  for(uint i=0; i<x_buf.size(); i++)
  {
    x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
    x_buf[i].R = es0.R.transpose() * x_buf[i].R;
  }

  pcl::PointCloud<PointType> pl_send, pl_path;
  int winsize = x_buf.size();
  for(int i=0; i<winsize; i++)
  {
    pcl::PointCloud<PointType> pl_tem = *pl_fulls[i];
    down_sampling_voxel(pl_tem, 0.01);
    pl_transform(pl_tem, x_buf[i]);
    pl_send += pl_tem;

    if((i%10==0 && i!=0) || i == winsize-1)
    {
      pub_pl_func(pl_send, pub_show);
      pl_send.clear();
      sleep(0.5);
    }

    PointType ap;
    ap.x = x_buf[i].p.x();
    ap.y = x_buf[i].p.y();
    ap.z = x_buf[i].p.z();
    ap.curvature = i;
    pl_path.push_back(ap);
  }

  pub_pl_func(pl_path, pub_path);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark2");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
  pub_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_show = n.advertise<sensor_msgs::PointCloud2>("/map_show", 100);
  pub_cute = n.advertise<sensor_msgs::PointCloud2>("/map_cute", 100);

  ros::Publisher br = n.advertise<tf2_msgs::TFMessage>("/tf", 1000);
  tf::TransformListener listener(n);

  string prename, ofname;
  vector<IMUST> x_buf;
  vector<pcl::PointCloud<PointType>::Ptr> pl_fulls;

  n.param<double>("voxel_size", voxel_size, 1);
  string file_path;
  n.param<string>("file_path", file_path, "");

  string bagfile;
  string trajectoryfile;
  nh.param<string>("bagfile", bagfile, "");
  nh.param<string>("trajectoryfile", trajectoryfile, "");
  string sensor_json;
  nh.param<string>("sensor_json", sensor_json, "");
  ROS_INFO("bagfile: %s, sensor_json: %s", bagfile.c_str(), sensor_json.c_str());

  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);

  int pose_size = 2001;
  int jump_num = 1;

  PLV(3) poss(pose_size);
  PLV(3) all_poss;
  PLM(3) rots(pose_size);
  PLM(3) all_rots;

  vector<string> topics;
  topics.push_back(string("/laser_horiz/clouds"));
  topics.push_back(string("/laser_vert/clouds"));
  topics.push_back(string("/imu"));
  topics.push_back(string("/tf"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::cout << "read scans, message view size: " << view.size() << std::endl;
  std::deque<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZI>>> lidar_buffer;
  pcl::PointCloud<pcl::PointXYZI> pc_horiz, pc_vert;

  ros::Time bag_begin_time = view.getBeginTime();
  ros::Time bag_end_time = view.getEndTime();
  tf2_ros::Buffer tf_buffer(ros::Duration(bag_end_time.toSec() - bag_begin_time.toSec() + 1));
  ReadStaticTransformsFromJson(sensor_json, &tf_buffer);

  //read and store all scans
  for(rosbag::MessageInstance const msg: view)
  {
      if (msg.getTopic() == "/laser_horiz/clouds")
      {
          static int horiz_num = 0;
          sensor_msgs::PointCloud2::ConstPtr scan = msg.instantiate<sensor_msgs::PointCloud2>();
          pcl::PointCloud<pcl::PointXYZI> pc;
          pcl::fromROSMsg(*scan, pc);
          pc_horiz += pc;
          pc_horiz.header.frame_id = "laser_horiz";
          ++horiz_num;
          if (horiz_num < 38) continue;
          lidar_buffer.push_back({scan->header.stamp, pc_horiz});
          pc_horiz.clear();
          horiz_num = 0;
      }
      else if (msg.getTopic() == "/laser_vert/clouds")
      {
          static int vert_num = 0;
          sensor_msgs::PointCloud2::ConstPtr scan = msg.instantiate<sensor_msgs::PointCloud2>();
          pcl::PointCloud<pcl::PointXYZI> pc;
          pcl::fromROSMsg(*scan, pc);
          pc_vert += pc;
          pc_vert.header.frame_id = "laser_vert";
          ++vert_num;
          if (vert_num < 38) continue;
          lidar_buffer.push_back({scan->header.stamp, pc_vert});
          pc_vert.clear();
          vert_num = 0;
      }
  }
  bag.close();

  //read trajectory
  std::cout << "read trajectory ..." << std::endl;
  std::vector<pcl::PointCloud<PointType>::Ptr> lidar_buffer2;

  rosbag::Bag trajectory_bag;
  trajectory_bag.open(trajectoryfile, rosbag::bagmode::Read);
  topics.clear();
  topics.push_back(string("/tf"));
  topics.push_back(string("trajectory_0"));
  rosbag::View view_tf(trajectory_bag, rosbag::TopicQuery(topics));

  // tf2buffer.lookupTransform("map", "trajectory_0", ros::Time(0));
  for (rosbag::MessageInstance const msg : view_tf) {
      if (lidar_buffer.empty()) break;
      if (msg.getTopic() == "/tf" )
      {
          tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
          tf_buffer.setTransform(tf_msg->transforms[0], "default_authority");
          br.publish(tf_msg);
      }
      else if (msg.getTopic() == "trajectory_0")
      {
          geometry_msgs::TransformStamped::ConstPtr tf_msg = msg.instantiate<geometry_msgs::TransformStamped>();
          tf_buffer.setTransform(*tf_msg, "default_authority");
      }
  }
  int lidar_count = 0;
  for (auto& ppc : lidar_buffer) {
      ++lidar_count;
      pcl::PointCloud<pcl::PointXYZI> cloud_body;
      tf2::Transform transform;
      try {
          auto geotf = tf_buffer.lookupTransform("map", "imu_frame", ppc.first);
          auto transformStamped = tf_buffer.lookupTransform("imu_frame", ppc.second.header.frame_id, ppc.first);
          Eigen::Isometry3d aff;
          aff = tf2::transformToEigen(transformStamped);
          pcl::transformPointCloud(ppc.second, cloud_body, aff.matrix());
          tf2::fromMsg(geotf.transform, transform);
      }
      catch(tf::TransformException &ex)
      {
          ROS_WARN("lookupTransform error: %s", ex.what());
          continue;
      }
      Eigen::Quaterniond q(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
      all_rots.push_back(q.toRotationMatrix());
      auto t = transform.getOrigin();
      all_poss.push_back(Eigen::Vector3d(t.x(), t.y(), t.z()));
      // std::cout << "index: " << lidar_count << ", t: " << t.x() << ", " << t.y() << ", " << t.z() << ", \nR: " << q.toRotationMatrix() << std::endl;
      pcl::PointCloud<PointType> pc;
      pc.header.frame_id = "imu";
      for (auto& point : cloud_body) {
          PointType p;
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          p.intensity = point.intensity;
          p.normal_x = 0;
          p.normal_y = 0;
          p.normal_z = 0;
          pc.push_back(p);
      }
      lidar_buffer2.push_back(pc.makeShared());
  }
  x_buf.resize(pose_size);
  for (int i = 0; i < rots.size(); ++i) {
      rots[i] << all_rots[i];
      poss[i] << all_poss[i];
      x_buf[i].R = rots[i];
      x_buf[i].p = poss[i];
  }

  IMUST es0 = x_buf[0];
  for(uint i=0; i<x_buf.size(); i++)
  {
    x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
    x_buf[i].R = es0.R.transpose() * x_buf[i].R;
  }

  win_size = x_buf.size();
  printf("The size of poses: %d\n", win_size);

  pl_fulls = lidar_buffer2;
  data_show(x_buf, pl_fulls);
  printf("Check the point cloud with the initial poses.\n");
  printf("If no problem, input '1' to continue or '0' to exit...\n");
  int a; cin >> a; if(a==0) exit(0);

  pcl::PointCloud<PointType> pl_full, pl_surf, pl_path, pl_send;
  for(int iterCount=0; iterCount<1; iterCount++)
  { 
    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

    eigen_value_array[0] = 1.0 / 16;
    eigen_value_array[1] = 1.0 / 16;
    eigen_value_array[2] = 1.0 / 9;

    for(int i=0; i<win_size; i++)
      cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i);

    pcl::PointCloud<PointType> pl_send;
    pub_pl_func(pl_send, pub_show);

    pcl::PointCloud<PointType> pl_cent; pl_send.clear();
    VOX_HESS voxhess;
    for(auto iter=surf_map.begin(); iter!=surf_map.end() && n.ok(); iter++)
    {
      iter->second->recut(win_size);
      iter->second->tras_opt(voxhess, win_size);
      iter->second->tras_display(pl_send, win_size);
    }

    pub_pl_func(pl_send, pub_cute);
    printf("\nThe planes (point association) cut by adaptive voxelization.\n");
    printf("If the planes are too few, the optimization will be degenerated and fail.\n");
    printf("If no problem, input '1' to continue or '0' to exit...\n");
    int a; cin >> a; if(a==0) exit(0);
    pl_send.clear(); pub_pl_func(pl_send, pub_cute);

    std::cout << "voxhess size: " << voxhess.plvec_voxels.size() << std::endl;
    if(voxhess.plvec_voxels.size() < x_buf.size())
    {
      printf("Initial error too large.\n");
      printf("Please loose plane determination criteria for more planes.\n");
      printf("The optimization is terminated.\n");
      exit(0);
    }

    BALM2 opt_lsv;
    opt_lsv.damping_iter(x_buf, voxhess);

    for(auto iter=surf_map.begin(); iter!=surf_map.end();)
    {
      delete iter->second;
      surf_map.erase(iter++);
    }
    surf_map.clear();

    malloc_trim(0);
  }

  printf("\nRefined point cloud is publishing...\n");
  malloc_trim(0);
  data_show(x_buf, pl_fulls);
  printf("\nRefined point cloud is published.\n");
  pcl::PointCloud<PointType> full_map, temp;
  for (int i = 0; i < x_buf.size(); ++i) {
      auto& it = pl_fulls.at(i);
      Eigen::Matrix4d aff = Eigen::Matrix4d::Identity();
      aff.topLeftCorner(3,3) = x_buf[i].R;
      aff.topRightCorner(3,1) = x_buf[i].p;
      pcl::transformPointCloud(*it, temp, aff);
      full_map += temp;
      temp.clear();
  }
  pcl::io::savePCDFileASCII("/home/south/calib.pcd", full_map);

  ros::spin();
  return 0;

}


