#include "ekf.hpp"

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle node;

    double spin_rate;

    //imposta rate
    node.param("spin",spin_rate, 1.0);

    ROSNode k(node,spin_rate);

    ros::spin();

}
