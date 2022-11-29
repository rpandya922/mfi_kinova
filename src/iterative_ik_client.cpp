#include <ros/ros.h>
#include "mfi_kinova/iterative_ik.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_client");
    // TODO: handle arguments properly

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mfi_kinova::iterative_ik>("ik_solver");
    mfi_kinova::iterative_ik srv;
    srv.request.px = atof(argv[1]);
    srv.request.py = atof(argv[2]);
    srv.request.pz = atof(argv[3]);
    srv.request.ow = atof(argv[4]);
    srv.request.ox = atof(argv[5]);
    srv.request.oy = atof(argv[6]);
    srv.request.oz = atof(argv[7]);

    if (client.call(srv))
    {
        ROS_INFO("Computed IK");
    }
    else
    {
        ROS_ERROR("Failed to call service iterative_ik");
        return 1;
    }

    return 0;
}
