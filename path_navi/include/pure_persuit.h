/*
@Time    : 2023 2023/6/28 下午9:45
@Author  : ZHANG WEN HAO
@Contact : 821298794@qq.com
@Version : 0.1
@Language: c++
@Desc    : 路径跟踪算法编写
*/

#ifndef _PURE_PURSUIT_H_
#define _PURE_PURSUIT_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <angles/angles.h>

class PurePursuit
{
public:
    PurePursuit();
    double CalculateDistance(double x1, double y1, double x2, double y2);
    bool SetGoalPath(nav_msgs::Path & msg);
    double ComputeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool GetCurrentPose(geometry_msgs::PoseStamped current_pose);
    int FindClosestWaypoint(nav_msgs::Path& path_msg, double x, double y);
    double CalculateAngleDifference(double current_angle, double target_angle);
    double getPoseYaw(const geometry_msgs::Pose& pose);
    bool isGoalReached();
    int FindClosestWaypoint(nav_msgs::Path goal_path,geometry_msgs::PoseStamped current_pose);
public:
    double ld_;
    int idx_ ;
    double position_tolerance_,rotation_tolerance_;
    double track_width_;
    double w_max_,w_min_,v_max_,v_min_,max_angle_difference_;


private:
    nav_msgs::Path goal_path_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped look_ahead_;
    bool goal_reached_,rotation_flag_;
    bool system_init_;
};


#endif
