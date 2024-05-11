/*
@Time    : 2023 2023/6/28 下午11:20
@Author  : ZHANG WEN HAO
@Contact : 821298794@qq.com
@Version : 0.1
@Language: c++
@Desc    : 路径跟随算法
*/
#ifndef PATH_NAVI_PATH_NAV_SERVER_H
#define PATH_NAVI_PATH_NAV_SERVER_H
#include <actionlib/server/simple_action_server.h>
#include <path_navi/PathNaviAction.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "pure_persuit.h"
#include <cstdlib>
#include "std_msgs/Bool.h"
#include "yaml-cpp/yaml.h"
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <mutex>

class PathNaviServer
{
public:
    PathNaviServer();
    void executeAction(const path_navi::PathNaviGoalConstPtr& goal_path);
private:
    void carStopCallback(const std_msgs::Bool::ConstPtr& msg);
    void carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& car_pose);
    void powerOffCallback(const std_msgs::Bool::ConstPtr& msg);
    bool getCurrentPose();
    void gnssStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    void remoteStopCallback(const std_msgs::Bool::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<path_navi::PathNaviAction> server_;
    tf::TransformListener listener_;  //tf树监听
    tf::StampedTransform transform_;  //tf树转换
    YAML::Node config_; //系统参数调整
    ros::Subscriber car_pose_sub_,poweroff_sub_,location_sub_,carstop_sub_,gnss_status_sub_,remotestop_sub_;
    ros::Publisher path_navi_vel_pub_  ,goal_path_pub_,current_path_pub_,local_pub_;
    std::mutex m_stop_,m_car_pose_;
    geometry_msgs::PoseStamped curr_location_; //当前位姿
    bool stop_flag_,gnss_stop_flag_,remote_stop_flag_;
    PurePursuit  PurePersuit_controller_;
};


#endif //PATH_NAVI_PATH_NAV_SERVER_H
