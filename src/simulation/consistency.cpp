#include "toolss.hpp"
// #include "BAs.hpp"
#include "BAs_left.hpp"
#include "sensor_json_parser.h"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <random>
#include <deque>

using namespace std;

template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
    pl.height = 1; pl.width = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pl, output);
    output.header.frame_id = "camera_init";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

void pub_odom_func(IMUST &xcurr)
{
    Eigen::Quaterniond q_this(xcurr.R);
    Eigen::Vector3d t_this(xcurr.p);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_this.x(), t_this.y(), t_this.z()));
    q.setW(q_this.w());
    q.setX(q_this.x());
    q.setY(q_this.y());
    q.setZ(q_this.z());
    transform.setRotation(q);
    ros::Time ct = ros::Time::now();
    br.sendTransform(tf::StampedTransform(transform, ct, "/camera_init", "/aft_mapped"));
}

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
    geometry_msgs::TransformStamped imuTransform;
    imuTransform.transform =
        ToGeometryMsgTransform(
            Eigen::Vector3d(imu.position[0], imu.position[1], imu.position[2]),
            Eigen::Quaterniond(imu.orientation[3], imu.orientation[0],
                               imu.orientation[1], imu.orientation[2]));

    imuTransform.child_frame_id = imu.name;
    imuTransform.header.frame_id = "base_link";
    tf_buffer->setTransform(imuTransform, "sensor_json", true);
    transforms.push_back(imuTransform);

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

    return transforms;
}

ros::Publisher pub_test, pub_curr, pub_full, pub_path;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simu2");
    ros::NodeHandle n;

    pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
    pub_curr = n.advertise<sensor_msgs::PointCloud2>("/map_curr", 100);
    pub_full = n.advertise<sensor_msgs::PointCloud2>("/map_full", 100);
    pub_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);

    ros::Publisher br = n.advertise<tf2_msgs::TFMessage>("/tf", 1000);
    tf::TransformListener listener(n);

    n.param<double>("pnoise", pnoise, 0.02);
    string file_path;
    n.param<string>("file_path", file_path, "");
    string bagfile;
    n.param<string>("bagfile", bagfile, "");
    string sensor_json;
    n.param<string>("sensor_json", sensor_json, "");
    ROS_INFO("bagfile: %s, sensor_json: %s", bagfile, sensor_json);

    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);
    double last_timestamp_lidar = -1;
    deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
    double last_tiemstamp_imu = -1;

    int pose_size = 101;
    fstream inFile(file_path + "/datas/consistency/lidarPose.csv", ios::in);
    int jump_num = 1;

    string lineStr, str;
    PLV(3) poss(pose_size);
    PLM(3) rots(pose_size);
    Eigen::Matrix4d aff;
    vector<double> nums;

    for(int i=0; i<pose_size; i++)
    {
        nums.clear();
        for(int j=0; j<4; j++)
        {
            getline(inFile, lineStr);
            stringstream ss(lineStr);
            while(getline(ss, str, ','))
                nums.push_back(stod(str));
        }

        for(int j=0; j<16; j++)
            aff(j) = nums[j];

        Eigen::Matrix4d affT = aff.transpose();

        static Eigen::Vector3d pos0 = affT.block<3, 1>(0, 3);

        rots[i] = affT.block<3, 3>(0, 0);
        poss[i] = affT.block<3, 1>(0, 3) - pos0;
    }

    vector<string> topics;
    topics.push_back(string("/laser_horiz/clouds"));
    topics.push_back(string("/imu"));
    topics.push_back(string("/tf"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::cout << "message view size: " << view.size() << std::endl;

    ros::Rate rate(10);
    pcl::PointCloud<pcl::PointXYZ> pl_orig;
    pcl::PointCloud<PointType> pl_full, pl_surf, pl_path, pl_send, pl_send2;

    sleep(1.5);
    for(int iterCount=0; iterCount<1 && n.ok(); iterCount++)
    {
        int win_count = 0;
        unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
        vector<IMUST> x_buf(win_size+fix_size);

        printf("Show the point cloud generated by simulator...\n");
        int m = 0;
        static tf::StampedTransform global_pose;
        global_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        global_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        global_pose.frame_id_ = "map";
        global_pose.child_frame_id_ = "base_link";
        tf::Transform diff;
        for(rosbag::MessageInstance const msg: view)
        {
            if (m == 0) global_pose.stamp_ = msg.getTime();
            bool updated = false;
            if (msg.getTopic() == "/tf")
            {
                tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
                br.publish(tf_msg);
                global_pose.stamp_ = tf_msg->transforms.front().header.stamp;
                updated = true;
            }
            else if (msg.getTopic() == "/laser_horiz/clouds")
            {
                sensor_msgs::PointCloud2::ConstPtr scan = msg.instantiate<sensor_msgs::PointCloud2>();
                tf::StampedTransform transform;
                listener.lookupTransform("base_link", "laser_horiz", ros::Time(0), transform);
                Eigen::Quaterniond q(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
                x_buf[win_count-1].R = q.toRotationMatrix();
                auto t = transform.getOrigin();
                x_buf[win_count-1].p = Eigen::Vector3d(t.x(), t.y(), t.z());
                ++m;
                pl_surf.clear();
                pcl::PCLPointCloud2 pc;
                pcl_conversions::copyPointCloud2MetaData(*scan, pc);
                pcl::cop
            }
            else if (msg.getTopic() == "/imu")
            {
                if (updated)
                {
                    sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
                    auto t1 = global_pose.stamp_;
                    auto t2 = imu_msg->header.stamp;
                    auto dt = (t2 - t1).toSec();
                    if (dt > 0)
                    {
                        auto dp = tf::Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                        auto dr = tf::Quaternion(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z) - global_pose.getRotation();
                    }
                }
            }

            pl_surf.clear();
            //pcl::io::loadPCDFile(filename, pl_surf);

            win_count++;
            x_buf[win_count-1].R = rots[m];
            x_buf[win_count-1].p = poss[m];

            pl_send = pl_surf;
            pl_transform(pl_send,x_buf[win_count-1].R, x_buf[win_count-1].p);
            PointType ap;
            ap.x = x_buf[win_count-1].p[0];
            ap.y = x_buf[win_count-1].p[1];
            ap.z = x_buf[win_count-1].p[2];
            pl_path.push_back(ap);
            pub_pl_func(pl_path, pub_path);
            pub_pl_func(pl_send, pub_curr);
            pub_pl_func(pl_send, pub_full);
            pub_odom_func(x_buf[win_count-1]);
            rate.sleep();

            cut_voxel(surf_map, pl_surf, x_buf[win_count-1], win_count-1);

            if(win_count < win_size + fix_size) continue;

            vector<IMUST> x_buf2;
            for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
            {
                iter->second->recut(win_count);
                iter->second->marginalize(fix_size, x_buf2, win_count);
            }

            x_buf2.resize(win_size);
            for(int i=0; i<win_size; i++)
                x_buf2[i] = x_buf[fix_size + i];
            x_buf = x_buf2;

            win_count -= fix_size;

            printf("The size of poses: %d\n", win_count);

            default_random_engine e(ros::Time::now().toSec());
            // default_random_engine e(200);

            /* Corrept the point cloud with random noise */
            for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
                iter->second->corrupt(e, x_buf, win_count);

            VOX_HESS voxhess;
            for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
                iter->second->tras_opt(voxhess, win_count);

            printf("Begin to optimize...\n");

            Eigen::MatrixXd Rcov(6*win_size, 6*win_size); Rcov.setZero();
            BALM2 opt_lsv;
            opt_lsv.damping_iter(x_buf, voxhess, Rcov);

            for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
                delete iter->second;
            surf_map.clear();

            Eigen::VectorXd err(6*win_size); err.setZero();
            for(int i=0; i<win_size; i++)
            {
                // err.block<3, 1>(6*i, 0) = Log(x_buf[i].R.transpose() * x_buf2[i].R);
                // err.block<3, 1>(6*i+3, 0) = (x_buf2[i].p - x_buf[i].p);
                err.block<3, 1>(6*i, 0) = Log(x_buf2[i].R * x_buf[i].R.transpose());
                err.block<3, 1>(6*i+3, 0) = -x_buf2[i].R * x_buf[i].R.transpose() * x_buf[i].p + x_buf2[i].p;
            }

            double nees = err.transpose() * Rcov.inverse() * err;
            printf("The expected NEES is 6*%d = %d.\n", win_size, 6*win_size);
            printf("The NEES for this Monto-Carlo experiment is %lf.\n", nees);

            /* 3 sigma bounds */
            // for(int i=0; i<win_size; i++)
            // {
            //   cout << i << ", " << err(6*i) << ", " << err(6*i+1) << ", " << err(6*i+2) << ", " << err(6*i+3) << ", " << err(6*i+4) << ", " << err(6*i+5) << ", ";
            //   cout << sqrt(Rcov(6*i, 6*i)) << "," << sqrt(Rcov(6*i+1, 6*i+1)) << "," << sqrt(Rcov(6*i+2, 6*i+2)) << "," << sqrt(Rcov(6*i+3, 6*i+3)) << "," << sqrt(Rcov(6*i+4, 6*i+4)) << "," << sqrt(Rcov(6*i+5, 6*i+5)) << endl;
            // }

            /* NEES for each pose (Require multiple Monte-Carlo experiments) */
            // for(int i=0; i<win_size; i++)
            // {
            //   Eigen::Matrix<double, 6, 1> err6 = err.block<6, 1>(6*i, 0);
            //   Eigen::Matrix<double, 6, 6> Rcov6 = Rcov.block<6, 6>(6*i, 6*i);
            //   double nees = err6.transpose() * Rcov6.inverse() * err6;
            //   double neesR = err6.head(3).transpose() * Rcov6.block<3, 3>(0, 0).inverse() * err6.head(3);
            //   double neesT = err6.tail(3).transpose() * Rcov6.block<3, 3>(3, 3).inverse() * err6.tail(3);
            //   cout << i << " " << nees << " " << neesR << " " << neesT << endl;
            // }

            break;
        }

    }

    ros::spin();
}

