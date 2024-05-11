/*
@Time    : 2023 2023/6/28 下午11:20
@Author  : ZHANG WEN HAO
@Contact : 821298794@qq.com
@Version : 0.1
@Language: c++
@Desc    : 路径跟随算法
*/
#include "path_navi_server.h"

PathNaviServer::PathNaviServer():server_(nh_, "iimt/path_navi", boost::bind(&PathNaviServer::executeAction, this, _1), false),stop_flag_(false),gnss_stop_flag_(false),remote_stop_flag_(false)
{
    server_.start();
    ROS_INFO("SERVER START !!!");
    //系统各项充电参数获取
    ros::NodeHandle nh("~");
    std::string configPath;
    nh.param<std::string>("config_path", configPath, "/home/iimt/robotconfig/system_param.yaml");
    config_ = YAML::LoadFile(configPath);

    PurePersuit_controller_.position_tolerance_ = config_["position_tolerance"].as<double>();
    PurePersuit_controller_.ld_ = config_["lookahead_distance"].as<double>();
    PurePersuit_controller_.track_width_ = config_["track_width"].as<double>();
    PurePersuit_controller_.w_max_ = config_["max_rotation_vel"].as<double>();
    PurePersuit_controller_.w_min_ = config_["min_rotation_vel"].as<double>();
    PurePersuit_controller_.v_max_ = config_["max_linear_vel"].as<double>();
    PurePersuit_controller_.v_min_ = config_["min_linear_vel"].as<double>();
    PurePersuit_controller_.max_angle_difference_ = config_["max_angle_difference"].as<double>();
    PurePersuit_controller_.rotation_tolerance_ = config_["rotation_tolerance"].as<double>();

    //监听tf树机器人状态
    listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));

    //话题发布和订阅
    gnss_status_sub_ = nh_.subscribe("/gnss_status", 10, &PathNaviServer::gnssStatusCallback, this); //定位状态
    carstop_sub_ = nh_.subscribe("/car_stop", 10, &PathNaviServer::carStopCallback, this); //导航途中遇障碍物停止
    remotestop_sub_ = nh_.subscribe("/remote_stop", 10, &PathNaviServer::remoteStopCallback, this); //导航途中遇障碍物停止
    car_pose_sub_ = nh_.subscribe("/car_pose", 1000, &PathNaviServer::carPoseCallback, this); //机器人位姿获取
    path_navi_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);   //发布速度
    goal_path_pub_ = nh_.advertise<nav_msgs::Path>("/navi_goal_path", 10);
    current_path_pub_ = nh_.advertise<nav_msgs::Path>("/navi_current_path", 10);

}


//获取机器人位姿获取
void PathNaviServer::carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& car_pose)
{
    m_car_pose_.lock();
    curr_location_.pose.position.x = car_pose->pose.position.x;
    curr_location_.pose.position.y = car_pose->pose.position.y;
    curr_location_.pose.orientation = car_pose->pose.orientation;
    m_car_pose_.unlock();
}


//得到当前位姿, 仿真时使用tf树位姿获取
bool PathNaviServer::getCurrentPose()
{
    try {
        listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
        curr_location_.header.frame_id = "map";
        curr_location_.header.stamp = ros::Time::now();
        curr_location_.pose.position.x = transform_.getOrigin().x();
        curr_location_.pose.position.y = transform_.getOrigin().y();
        curr_location_.pose.orientation.x = transform_.getRotation().x();
        curr_location_.pose.orientation.y = transform_.getRotation().y();
        curr_location_.pose.orientation.z = transform_.getRotation().z();
        curr_location_.pose.orientation.w = transform_.getRotation().w();
        return true;
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}


//机器人停止前进
void PathNaviServer::carStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    stop_flag_ = msg->data;
}

//机器人停止前进
void PathNaviServer::remoteStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    remote_stop_flag_ = msg->data;
}

//gnss状态差停止运行
void PathNaviServer::gnssStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    gnss_stop_flag_ = msg->data;
}


//关机操作
void PathNaviServer::powerOffCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        std::cout<<"power_off"<<std::endl;
        system("shutdown -h now");
    }
}


//action回调
void PathNaviServer::executeAction(const path_navi::PathNaviGoalConstPtr& goal_path_msg)
{
    ROS_INFO("GET PATH NAVI !!!");
    double distance = 0.0;
    geometry_msgs::Twist cmd_vel;
    nav_msgs::Path base_link_path;
    //设置当前目标路径
    nav_msgs::Path goal_path = goal_path_msg->path;
    goal_path.header.stamp = ros::Time::now(); // 设置时间戳
    goal_path.header.frame_id = "map";
    bool set_path_ret = PurePersuit_controller_.SetGoalPath(goal_path);
    if(set_path_ret == false)
    {
        ROS_WARN_STREAM("Set path Failed! Ensure it not empty!");
        return;
    }
    ros::Rate r(10);
    bool navi_result = false;

    while (!navi_result && ros::ok()) {
        r.sleep();
        //发布当前目标路径
        goal_path_pub_.publish(goal_path);

        //仿真时开启（获取当前位姿）
        //bool ret = getCurrentPose();
       // if (!ret)
      //  {
      //      ROS_INFO("GET CURRENT POSE FAILED !!!");
      ////      continue;
       // }

        //发布当前行走轨迹
        base_link_path.header.stamp = ros::Time::now(); // 设置时间戳
        base_link_path.header.frame_id = "map"; // 设置坐标系，这里假设路径是在"map"坐标系下
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position = curr_location_.pose.position;
        base_link_path.poses.push_back(pose);
        current_path_pub_.publish(base_link_path);


        //目标点中断
        if (server_.isPreemptRequested()) {
            ROS_INFO("Cancelled!!!");
            path_navi::PathNaviResult result;
            result.goal_reached = false;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            path_navi_vel_pub_.publish(cmd_vel);
            server_.setPreempted();
            return;
        }

        //控制器获取当前位姿
        m_car_pose_.lock();
        bool ret_get_pose = PurePersuit_controller_.GetCurrentPose(curr_location_);
        m_car_pose_.unlock();
        if(ret_get_pose == false)
        {
            ROS_INFO("FAILED TO GET CURRENT POSE!!!");
            continue;
        }

        if(remote_stop_flag_)
        {
            remote_stop_flag_ = false;
            ROS_INFO("remote stop!!!");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z  = 0;
            path_navi_vel_pub_.publish(cmd_vel);
            continue;
        }

        if(gnss_stop_flag_)
        {
            ROS_INFO("GPS status error!!!");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z  = 0;
            path_navi_vel_pub_.publish(cmd_vel);
            continue;
        }

        //雷达是否被遮挡住，被遮挡住则置为true，速度为0
        if(stop_flag_)
        {
            ROS_INFO("It's blocked!!!");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z  = 0;
            path_navi_vel_pub_.publish(cmd_vel);
            continue;
        }


        distance = PurePersuit_controller_.ComputeVelocityCommands(cmd_vel);
        path_navi_vel_pub_.publish(cmd_vel);

        //达到目标点
        if (PurePersuit_controller_.isGoalReached()) {
            goal_path.poses.clear();
            navi_result = true;
            break;
        }

        //实时给出运动反馈
        path_navi::PathNaviFeedback feedback;
        feedback.distance = distance;
        feedback.goal_reached = false;
        server_.publishFeedback(feedback);
    }
    //跳出，如何存在结果则认为到达目标点
    if(navi_result)
    {
        //发布反馈以及最终结果
        path_navi::PathNaviFeedback feedback;
        feedback.distance = distance;
        feedback.goal_reached = true;
        server_.publishFeedback(feedback);

        path_navi::PathNaviResult result;
        result.goal_reached = true;
        server_.setSucceeded(result);
        ROS_INFO("ARRIVED GOAL!!!");
    }
    else
    {
        ROS_INFO("ROS NOT CONNECTED!!!");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_navi_server");

    PathNaviServer server;

    ros::spin();

    return 0;
}
