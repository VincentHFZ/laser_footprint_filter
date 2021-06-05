#ifndef LASER_FOOTPRINT_FILTER_H
#define LASER_FOOTPRINT_FILTER_H

#include <iostream>
#include <string>
#include <cmath>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>
#include <Eigen/Eigen>

bool getSensorPose(tf::TransformListener* p_tf_, Eigen::Vector3d& pos);
void filterLaserByFootPrint(Eigen::Vector3d sensor_pose, double robot_raidus, champion_nav_msgs::ChampionNavLaserScan& scan);

#endif // LASER_FOOTPRINT_FILTER_H
