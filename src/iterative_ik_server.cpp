#include <ros/ros.h>
#include "mfi_kinova/iterative_ik.h"

#include "iterative_ik_server.hpp"

#include "KinovaGen3.hpp"

bool iterative_ik(mfi_kinova::iterative_ik::Request &req,
                  mfi_kinova::iterative_ik::Response &res_)
{
    Eigen::Vector3f transl = Eigen::Vector3f(req.px, req.py, req.pz);
    Eigen::Matrix3f rot = Eigen::Quaternionf(req.ow, req.ox, req.oy, req.oz).normalized().toRotationMatrix();
    Vector7f q_curr;
    q_curr << req.j1, req.j2, req.j3, req.j4, req.j5, req.j6, req.j7;

    Eigen::Matrix4f pEE_des; // desired end effector pose
    pEE_des.setIdentity();
    pEE_des.block<3,3>(0,0) = rot; // set rotational component
    pEE_des.block<3,1>(0,3) = transl; // set translational component

    // std::cout << "T=" << std::endl << pEE_des << std::endl;

    Eigen::VectorXf q_output;

    // construct KinovaGen3 object
    const std::string ip_addr{"192.168.1.10"};
    const std::string username{"admin"};
    const std::string passwd{"admin"};
    const int         port{10000};
    const int         port_realtime{10001};
    const bool low_level_control = true;
    auto arm_ptr = std::make_shared<rc::KinovaGen3>(ip_addr, port, port_realtime, username, passwd, low_level_control);

    ROS_INFO("Starting IK computation.");
    arm_ptr->iterative_ik(pEE_des, q_output, q_curr);

    res_.j1 = q_output(0);
    res_.j2 = q_output(1);
    res_.j3 = q_output(2);
    res_.j4 = q_output(3);
    res_.j5 = q_output(4);
    res_.j6 = q_output(5);
    res_.j7 = q_output(6);

    ROS_INFO("IK computation finished.");
    // delete &arm_ptr;
    return true;
}

int main(int argc, char **argv)
{
    // initialize link_transform
    // GenerateLinkTransform();

    ros::init(argc, argv, "ik_server");
    ros::NodeHandle n;

    // // construct KinovaGen3 object
    // const std::string ip_addr{"192.168.1.10"};
    // const std::string username{"admin"};
    // const std::string passwd{"admin"};
    // const int         port{10000};
    // const int         port_realtime{10001};
    // const bool low_level_control = true;

    // rc::KinovaGen3 arm_ptr = rc::KinovaGen3(ip_addr, port, port_realtime, username, passwd, low_level_control);
    // auto arm_ptr = std::make_shared<rc::KinovaGen3>(ip_addr, port, port_realtime, username, passwd, low_level_control);

    // auto ik = std::bind(iterative_ik, arm_ptr, _1, _2);

    ros::ServiceServer service = n.advertiseService("ik_solver", iterative_ik);
    ROS_INFO("Ready to compute IK.");
    ros::spin();

    return 0;
}
