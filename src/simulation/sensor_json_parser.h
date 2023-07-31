#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

using namespace boost::property_tree;

struct sensor_model {
    // common
    std::string name;
    std::string serial;
    double position[3];
    double orientation[4];
    // imu
    // todo : imu intrinsic
    //  camera
    double pol[1024];
    int length_pol;
    double invpol[64];
    int length_invpol;
    double xc;
    double yc;
    double c;
    double d;
    double e;
    int width;
    int height;
};

void parserPtree(boost::property_tree::ptree& pt, sensor_model& model, int type = 0) {
    try {
#pragma region common part
        // common part
        model.name = pt.get<std::string>("name");
        model.serial = pt.get<std::string>("serial");
        boost::property_tree::ptree positon = pt.get_child("position");
        boost::property_tree::ptree orientation = pt.get_child("orientation");
        int index = 0;
        for (auto iter = positon.begin(); iter != positon.end(); iter++) {
            model.position[index] = iter->second.get_value<double>();
            index++;
        }
        index = 0;
        for (auto iter = orientation.begin(); iter != orientation.end(); iter++) {
            model.orientation[index] = iter->second.get_value<double>();
            index++;
        }
#pragma endregion

        if (type == 1)  // camera
        {
            // pol
            boost::property_tree::ptree pol = pt.get_child("pol");
            int pol_l = 0;
            for (auto iter = pol.begin(); iter != pol.end(); iter++) {
                // model.pol.emplace_back(iter->second.get_value<double>());
                model.pol[pol_l] = iter->second.get_value<double>();
                pol_l++;
            }
            // model.length_pol = model.pol.size();
            model.length_pol = pol_l;
            // height width
            boost::property_tree::ptree imagesize = pt.get_child("imagesize");
            auto imagesize_iter = imagesize.begin();
            model.height = imagesize_iter->second.get_value<int>();
            imagesize_iter++;
            model.width = imagesize_iter->second.get_value<int>();
            // xc yc
            boost::property_tree::ptree center = pt.get_child("center");
            auto center_iter = center.begin();
            model.xc = center_iter->second.get_value<double>();
            center_iter++;
            model.yc = center_iter->second.get_value<double>();
            // c d e
            boost::property_tree::ptree stretchmatrix = pt.get_child("stretchmatrix");
            auto stretchmatrix_iter = stretchmatrix.begin();
            model.c = stretchmatrix_iter->second.get_value<double>();
            stretchmatrix_iter++;
            model.d = stretchmatrix_iter->second.get_value<double>();
            stretchmatrix_iter++;
            model.e = stretchmatrix_iter->second.get_value<double>();
            // invpol
            boost::property_tree::ptree invpol = pt.get_child("invpol");
            int invpol_l = 0;
            for (auto iter = invpol.begin(); iter != invpol.end(); iter++) {
                // model.invpol.emplace_back(iter->second.get_value<double>());
                model.invpol[invpol_l] = iter->second.get_value<double>();
                invpol_l++;
            }
            // model.length_invpol = model.invpol.size();
            model.length_invpol = invpol_l;
            std::reverse(model.invpol, model.invpol + model.length_invpol);  //反转inv 因为我matlab输出的是反的
        }                                                                    // end type1
        else if (type == 2)                                                  // imu
        {
        } else if (type == 3)  // lidar
        {
        }
    } catch (boost::property_tree::ptree_error& e) {
        std::cout << e.what() << std::endl;
        return;
    }
    return;
}

int parserIntrinsicJson(std::string jsonPath, sensor_model& cam0, sensor_model& cam1, sensor_model& cam2, sensor_model& cam3, sensor_model& lidar_horiz, sensor_model& lidar_vert, sensor_model& imu) {
    std::fstream ss(jsonPath);
    boost::property_tree::ptree root;
    try {
        boost::property_tree::read_json(ss, root);
        try {
            boost::property_tree::ptree c0 = root.get_child("cam0");
            parserPtree(c0, cam0, 1);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }
        try {
            boost::property_tree::ptree c1 = root.get_child("cam1");
            parserPtree(c1, cam1, 1);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }
        try {
            boost::property_tree::ptree c2 = root.get_child("cam2");
            parserPtree(c2, cam2, 1);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }
        try {
            boost::property_tree::ptree c3 = root.get_child("cam3");
            parserPtree(c3, cam3, 1);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }
        try {
            boost::property_tree::ptree lh = root.get_child("lidar_horiz");
            parserPtree(lh, lidar_horiz, 2);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }
        try {
            boost::property_tree::ptree lv = root.get_child("lidar_vert");
            parserPtree(lv, lidar_vert, 2);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }
        try {
            boost::property_tree::ptree imuTree = root.get_child("imu");
            parserPtree(imuTree, imu, 3);
        } catch (boost::property_tree::ptree_error& e) {
            std::cout << e.what() << std::endl;
        }

    } catch (boost::property_tree::ptree_error& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 1;
}
