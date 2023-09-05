#include "toolss.hpp"
// #include "BAs.hpp"
#include "BAs_left.hpp"
#include "sensor_json_parser.h"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
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
    output.header.frame_id = "map";
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
    br.sendTransform(tf::StampedTransform(transform, ct, "/map", "/base_link"));
}

ros::Publisher pub_test, pub_curr, pub_full, pub_path;

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
        pl_transform(pl_tem, x_buf[i].R, x_buf[i].p);
        pl_send += pl_tem;

        if((i%2==0 && i!=0) || i == winsize-1)
        {
            pub_pl_func(pl_send, pub_curr);
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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "simu2");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
    pub_curr = n.advertise<sensor_msgs::PointCloud2>("/map_curr", 100);
    pub_full = n.advertise<sensor_msgs::PointCloud2>("/map_full", 100);
    pub_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);

    ros::Publisher br = n.advertise<tf2_msgs::TFMessage>("/tf", 1000);
    tf::TransformListener listener(n);

    nh.param<double>("pnoise", pnoise, 0.02);
    string file_path;
    nh.param<string>("file_path", file_path, "");
    string bagfile;
    string trajectoryfile;
    nh.param<string>("bagfile", bagfile, "");
    nh.param<string>("trajectoryfile", trajectoryfile, "");
    string sensor_json;
    nh.param<string>("sensor_json", sensor_json, "");
    ROS_INFO("bagfile: %s, sensor_json: %s", bagfile.c_str(), sensor_json.c_str());

    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);

    int pose_size = 1201;
    int jump_num = 1;

    string lineStr, str;
    PLV(3) poss(pose_size);
    PLV(3) all_poss;
    //    PLV(3) all_poss_vert;
    PLM(3) rots(pose_size);
    PLM(3) all_rots;
    //    PLM(3) all_rots_vert;
    Eigen::Matrix4d aff;
    vector<double> nums;

    vector<string> topics;
    topics.push_back(string("/laser_horiz/clouds"));
    topics.push_back(string("/laser_vert/clouds"));
    topics.push_back(string("/imu"));
    topics.push_back(string("/tf"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::cout << "read scans, message view size: " << view.size() << std::endl;
    std::vector<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZI>>> lidar_buffer;
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
        /*else if (msg.getTopic() == "/laser_vert/clouds")
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
        }*/
    }
    bag.close();

    //read trajectory
    std::cout << "read trajectory ..." << std::endl;
    std::vector<pcl::PointCloud<PointType>::Ptr> lidar_buffer2;
    pcl::PointCloud<pcl::PointXYZI> lidar_bodys;

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
            auto global_tf = tf_buffer.lookupTransform("map", ppc.second.header.frame_id, ppc.first);
            Eigen::Isometry3d aff;
            aff = tf2::transformToEigen(global_tf);
            pcl::transformPointCloud(ppc.second, cloud_body, aff.matrix());
            lidar_bodys += cloud_body;

            cloud_body.clear();
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
    pcl::io::savePCDFileASCII("/home/south/result/calib_raw.pcd", lidar_bodys);
    lidar_bodys.clear();
    IMUST es0;
    es0.R = all_rots[0];
    es0.p = all_poss[0];
    rots[0] = es0.R;
    poss[0] = es0.p;
    for (int i = 0; i < rots.size(); ++i) {
        rots[i] << es0.R.transpose() * all_rots[i];
        poss[i] << es0.R.transpose() * (all_poss[i] - es0.p);
    }
    ros::Rate rate(20);
    pcl::PointCloud<pcl::PointXYZ> pl_orig;
    pcl::PointCloud<PointType> pl_full, pl_surf, pl_path, pl_send, pl_send2;

    sleep(1.5);
    for(int iterCount=0; iterCount<1 && n.ok(); iterCount++)
    {
        int win_count = 0;
        unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
        vector<IMUST> x_buf(win_size+fix_size);

        printf("Show the point cloud generated by rosbag...\n");
        for(int m = 0; m < lidar_buffer2.size(); ++m)
        {
            pl_surf.clear();
            //pcl::io::loadPCDFile(filename, pl_surf);
            pl_surf = *lidar_buffer2.at(m);

            win_count++;
            x_buf[win_count-1].R = rots[m];
            x_buf[win_count-1].p = poss[m];
            // std::cout << "p: " << x_buf[win_count-1].p << "\nR: " << x_buf[win_count-1].R << std::endl;

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
            std::cout << "The size of surf_map: " << surf_map.size() << std::endl;

            default_random_engine e(ros::Time::now().toSec());
            // default_random_engine e(200);

            /* Corrept the point cloud with random noise */
            // for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
            //     iter->second->corrupt(e, x_buf, win_count);

            VOX_HESS voxhess;
            for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++){
                iter->second->tras_opt(voxhess, win_count);
                // std::cout << "octo_state: " << iter->second->octo_state << ", push_state:" << iter->second->push_state << ", fix point size: " << iter->second->fix_point.N << std::endl;
            }

            printf("Begin to optimize...\n");

            Eigen::MatrixXd Rcov(6*win_size, 6*win_size); Rcov.setZero();
            BALM2 opt_lsv;
            std::cout << "voxhess size: " << voxhess.plvec_voxels.size() << std::endl;
            opt_lsv.damping_iter(x_buf, voxhess, Rcov, 0);

            /*for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
                delete iter->second;
            surf_map.clear();*/

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
            /*for(int i=0; i<win_size; i++)
            {
              cout << i << ", " << err(6*i) << ", " << err(6*i+1) << ", " << err(6*i+2) << ", " << err(6*i+3) << ", " << err(6*i+4) << ", " << err(6*i+5) << ", ";
              cout << sqrt(Rcov(6*i, 6*i)) << "," << sqrt(Rcov(6*i+1, 6*i+1)) << "," << sqrt(Rcov(6*i+2, 6*i+2)) << "," << sqrt(Rcov(6*i+3, 6*i+3)) << "," << sqrt(Rcov(6*i+4, 6*i+4)) << "," << sqrt(Rcov(6*i+5, 6*i+5)) << endl;
            }*/

            /* NEES for each pose (Require multiple Monte-Carlo experiments) */
            /*for(int i=0; i<win_size; i++)
            {
              Eigen::Matrix<double, 6, 1> err6 = err.block<6, 1>(6*i, 0);
              Eigen::Matrix<double, 6, 6> Rcov6 = Rcov.block<6, 6>(6*i, 6*i);
              double nees = err6.transpose() * Rcov6.inverse() * err6;
              double neesR = err6.head(3).transpose() * Rcov6.block<3, 3>(0, 0).inverse() * err6.head(3);
              double neesT = err6.tail(3).transpose() * Rcov6.block<3, 3>(3, 3).inverse() * err6.tail(3);
              cout << i << " " << nees << " " << neesR << " " << neesT << endl;
            }*/

            printf("\nRefined point cloud is publishing...\n");
            //malloc_trim(0);
            data_show(x_buf, lidar_buffer2);
            printf("\nRefined point cloud is published.\n");
            std::vector<pcl::PointCloud<pcl::PointXYZI>> lidar_buffer3;
            for (auto pair : lidar_buffer2) {
                pcl::PointCloud<pcl::PointXYZI> pc;
                for (auto it: pair->points) {
                    pcl::PointXYZI p;
                    p.x = it.x;
                    p.y = it.y;
                    p.z = it.z;
                    p.intensity = it.intensity;
                    pc.push_back(p);
                }
                lidar_buffer3.push_back(pc);
            }
            pcl::PointCloud<pcl::PointXYZI> full_map, temp;
            for (int i = 0; i < x_buf.size(); ++i) {
                auto& it = lidar_buffer3.at(i);
                Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
                Eigen::Matrix4d aff = Eigen::Matrix4d::Identity();
                aff.topLeftCorner(3,3) = x_buf[i].R;
                aff.topRightCorner(3,1) = x_buf[i].p;
                iso.matrix() = aff;
                pcl::transformPointCloud(it, temp, iso.matrix());
                full_map += temp;
                temp.clear();
            }
            for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
                delete iter->second;
            surf_map.clear();
            pcl::io::savePCDFileASCII("/home/south/result/calib" + std::to_string(iterCount) + ".pcd", full_map);

            break;
        }
    }

    ros::spin();
}

