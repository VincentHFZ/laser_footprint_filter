#include "laser_footprint_filter/laser_footprint_filter.h"

// 获取传感器相对于机器的位姿
bool getSensorPose(tf::TransformListener* p_tf_, Eigen::Vector3d& pos)
{

    tf::StampedTransform tmp_transform;
    double dx, dy, theta;
    try {
        if(p_tf_->waitForTransform("/base_link", "/base_laser", ros::Time(), ros::Duration(0.1)) == false)
        {
            std::cout << "Can not get transform from base_laser to base_link" << std::endl;
            std::cout << "No no no!!!" << std::endl;
        }
        p_tf_->lookupTransform("/base_link", "/base_laser", ros::Time(), tmp_transform);

        theta = tf::getYaw(tmp_transform.getRotation());
        tf::Vector3 origin = tmp_transform.getOrigin();
        dx = origin.getX();
        dy = origin.getY();
    }
    catch (const tf2::LookupException& ex)
    {
        std::cout << "Can not get transform from base_link to odom " << ex.what() << std::endl;
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        std::cout << "Can not get transform from base_link to odom " << ex.what() << std::endl;
        return false;
    }

    pos << dx, dy, theta;

    return true;
}

// 将原始数据转换到机器坐标系上
// 将大于半径的点另外存储为激光数据
void filterLaserByFootPrint(Eigen::Vector3d sensor_pose, double robot_radius, champion_nav_msgs::ChampionNavLaserScan &scan)
{
    champion_nav_msgs::ChampionNavLaserScan output_scan;

    double dx = sensor_pose[0];
    double dy = sensor_pose[1];
    double theta = sensor_pose[2];

    for(int i = 0; i < scan.ranges.size(); i++)
    {
        double range = scan.ranges[i];
        double angle = scan.angles[i];

        if(range > scan.range_max || range < scan.range_min || std::isinf(range) || std::isnan(range))
            continue;
        Eigen::Vector2d laserlink_pt;
        laserlink_pt << range * std::cos(angle), range * std::sin(angle);
        Eigen::Vector2d baselink_pt;
        baselink_pt[0] = laserlink_pt[0] * std::cos(theta) - laserlink_pt[1] * std::sin(theta) + dx;
        baselink_pt[1] = laserlink_pt[0] * std::sin(theta) + laserlink_pt[1] * std::cos(theta) + dy;

        if(baselink_pt[0] * baselink_pt[0] + baselink_pt[1] * baselink_pt[1] < robot_radius * robot_radius)
        // if(baselink_pt.norm() < robot_radius)
            scan.ranges[i] = scan.range_min - 0.01;

    }
}
