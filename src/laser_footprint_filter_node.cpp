#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include "laser_footprint_filter/laser_footprint_filter.h"

ros::Publisher scan_pub;
Eigen::Vector3d g_laser_pose;
double robot_radius = 0.3;

void getChampionScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& p_scan)
{
    champion_nav_msgs::ChampionNavLaserScan scan = *p_scan;
    filterLaserByFootPrint(g_laser_pose, 0.3, scan);

    scan_pub.publish(scan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LaserFootprintFilter");
    ros::NodeHandle nh;

    tf::TransformListener tf_listener(ros::Duration(1));
    ros::Duration(1.0).sleep();

    if(getSensorPose(&tf_listener, g_laser_pose) == false)
    {
        std::cout << "Error" << std::endl;
        ros::shutdown();
    }

    ros::Subscriber scan_sub = nh.subscribe("/sick_scan", 10, getChampionScanCallback);
    scan_pub = nh.advertise<champion_nav_msgs::ChampionNavLaserScan>("/filter_scan", 10, true);

    ros::spin();

    return 0;
}
