/*
@Time    : 2023 2023/6/28 下午9:45
@Author  : ZHANG WEN HAO
@Contact : 821298794@qq.com
@Version : 0.1
@Language: c++
@Desc    : 纯路径跟踪
*/
#include "pure_persuit.h"



PurePursuit::PurePursuit():position_tolerance_(0.1),rotation_tolerance_(0.1),goal_reached_(false),rotation_flag_(false),track_width_(0.3),w_max_(0.3),w_min_(0.1),v_max_(0.2),v_min_(0.05),ld_(1.0),idx_(0),max_angle_difference_(0.75),system_init_(false)
{

}

// 计算与目标路径点的距离
double PurePursuit::CalculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

//得到目标路径
bool PurePursuit::SetGoalPath(nav_msgs::Path& msg)
{
    system_init_ = false;
    idx_ = 0;
    goal_path_ = msg;
    rotation_flag_ = false;
    if (msg.poses.size() > 0)
    {
        goal_reached_ = false;
        ROS_INFO("set goal_path successful !!!");
        return true;
    }
    else
    {
        goal_reached_ = true;
        ROS_WARN_STREAM("Received empty path!");
        return false;
    }
}


//获取当前机器人位置信息
bool PurePursuit::GetCurrentPose(geometry_msgs::PoseStamped current_pose)
{
    current_pose_ = current_pose;
    return true;
}


double PurePursuit::getPoseYaw(const geometry_msgs::Pose& pose)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

// 计算车辆与目标路径点的角度差
double PurePursuit::CalculateAngleDifference(double current_angle, double target_angle) {
    double diff = target_angle - current_angle;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

int PurePursuit::FindClosestWaypoint(nav_msgs::Path goal_path,geometry_msgs::PoseStamped current_pose)
{
    // 初始化最小距离和最近点的ID
    double min_distance = std::numeric_limits<double>::max();
    int nearest_id = -1;
    // 遍历goal_path中的所有姿态
    for (size_t i = 0; i < goal_path.poses.size(); ++i) {
        const geometry_msgs::PoseStamped& pose = goal_path.poses[i];
        // 计算当前姿态与遍历到的姿态之间的距离
        double distance = std::sqrt(std::pow(pose.pose.position.x - current_pose.pose.position.x, 2) +
                                    std::pow(pose.pose.position.y - current_pose.pose.position.y, 2));

        // 如果找到更近的点，则更新最小距离和最近点的ID
        if (distance < min_distance) {
            min_distance = distance;
            nearest_id = i;
        }
    }
    return nearest_id;
}

//计算速度控制
double PurePursuit::ComputeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    int idx_end = goal_path_.poses.size() - 1; //最后一个位姿的索引
    cmd_vel.angular.z  = 0;
    cmd_vel.linear.x  = 0;
    //第一个点的时候，需要首先确定需要跟踪的点是哪个，然后在旋转过去。
    if(!system_init_)
    {
        ROS_INFO("SETTING START POINT!");
        //查找最接近机器人的路径点,设为最终的跟踪点
        int closest_idx = FindClosestWaypoint(goal_path_, current_pose_);
        
        if(closest_idx != -1)
        {
            idx_ = closest_idx;
        }
    }

    if(rotation_flag_ == false)
    {
        ROS_INFO("FOLLOWING PATH!");
        // 获取当前目标路径点坐标
        double target_x = goal_path_.poses[idx_].pose.position.x;
        double target_y = goal_path_.poses[idx_].pose.position.y;



        int i= 0;
        int closest_idx = FindClosestWaypoint(goal_path_, current_pose_);
        for(i = closest_idx; i<= idx_end ;i++ )
        {
                double distance_1 = CalculateDistance(goal_path_.poses[closest_idx].pose.position.x, goal_path_.poses[closest_idx].pose.position.y, goal_path_.poses[i].pose.position.x, goal_path_.poses[i].pose.position.y);
                if(distance_1>ld_)
                {
                     idx_ = i;
                     target_x = goal_path_.poses[idx_].pose.position.x;
                     target_y = goal_path_.poses[idx_].pose.position.y;
                     break;
                }
        }
        
        if(i == idx_end)
        {
             idx_ = idx_end;
             target_x = goal_path_.poses[idx_].pose.position.x;
             target_y = goal_path_.poses[idx_].pose.position.y;
        }
        // 计算车辆与目标路径点的距离
        double distance = CalculateDistance(current_pose_.pose.position.x, current_pose_.pose.position.y, target_x, target_y);
        double current_yaw = getPoseYaw(current_pose_.pose);
        // 计算车辆与目标路径点的角度差
        //double angle_difference = CalculateAngleDifference(current_yaw, std::atan2(target_y - current_pose_.pose.position.y, target_x - current_pose_.pose.position.x));

        int idx_average = idx_+5;
        if(idx_average>idx_end)
        {
           idx_average = idx_end;
        }
        double angle_difference =0;
        int infinte_value =0;
        for(int j = idx_; j < idx_average;j++ )
        {
	        if(goal_path_.poses[j].pose.position.x - current_pose_.pose.position.x<1e-6)
	        {
	            angle_difference += 0.0;
	            infinte_value++;
	            std::cout<<"position error!"<<std::endl;
	        }
	        else
	        {
	            angle_difference += CalculateAngleDifference(current_yaw, std::atan2(goal_path_.poses[j].pose.position.y - current_pose_.pose.position.y, goal_path_.poses[j].pose.position.x - current_pose_.pose.position.x));
	        }
	    }
	    angle_difference = angle_difference/(idx_average - (idx_ +infinte_value ));
        // 计算期望的线速度
        double linear_velocity = v_max_;
        linear_velocity *= distance / (ld_);
        std::cout<<"distance:"<<distance<<std::endl;

        // 限制线速度在设定的范围内
        if (linear_velocity > v_max_) {
            linear_velocity = v_max_;
        }
        else if (linear_velocity < v_min_) {
            linear_velocity = v_min_;
        }
        else
        {
            linear_velocity = 0;
        }

        cmd_vel.linear.x = linear_velocity;

        // 计算期望的角速度
        // cmd_vel.angular.z = 2 * cmd_vel.linear.x * std::sin(pf_angle_to_global_plan) / distance;
        double angular_velocity = 2.0 * cmd_vel.linear.x * std::sin(angle_difference) / ld_;
        std::cout<<"angle_difference:"<<angle_difference<<"cmd_vel.linear.x :"<<cmd_vel.linear.x <<std::endl;
        // 限制角速度在设定的范围内
        if (angular_velocity > w_max_) {
            angular_velocity = w_max_;
        }
        else if (angular_velocity < -w_max_) {
            angular_velocity = -w_max_;
        }
        else if (std::fabs(angular_velocity) < w_min_) {
            angular_velocity = w_min_ * double(std::abs(angular_velocity)/angular_velocity);
        }
        else
       {
           angular_velocity = 0.0;
       }
        cmd_vel.angular.z = angular_velocity;
        // 判断是否到达目标点
        if (idx_ == goal_path_.poses.size() - 1 && distance < position_tolerance_)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            rotation_flag_ = true;
            ROS_INFO("Reached the goal!");
            // 在此执行到达目标点后的操作
        }
    }
    else    //到达位置附近开始调整最终姿态
    {
        ROS_INFO("ADJUST POSE!");
        if(idx_ != idx_end)
        {
            ROS_INFO("ROTATION ERROR!");
        }
        //所需旋转的角度
        double current_yaw = getPoseYaw(current_pose_.pose);
        double final_yaw = getPoseYaw(goal_path_.poses[idx_end].pose);
        double final_angle_difference = CalculateAngleDifference(current_yaw, final_yaw);
        std::cout<<"current_yaw:"<<current_yaw<<"final_yaw:"<<final_yaw<<"final_angle_difference:"<<final_angle_difference<<std::endl;
        if(std::fabs(final_angle_difference) < rotation_tolerance_)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            goal_reached_ = true;
        }
        else
        {
            // 计算期望的角速度
            double angular_velocity = 2.0 * final_angle_difference / track_width_;
            // 限制角速度在设定的范围内
            if (angular_velocity > w_max_) {
                angular_velocity = w_max_;
            } else if (angular_velocity < -w_max_) {
                angular_velocity = -w_max_;
            } else if (std::fabs(angular_velocity) < w_min_) {
                angular_velocity = w_min_ * double(std::abs(angular_velocity)/angular_velocity);
            }
            else
            {
                angular_velocity = 0.0;
            }
            cmd_vel.angular.z = angular_velocity;
            cmd_vel.linear.x = 0.0; // 停止线速度
        }
    }
    //计算出距离最后一个轨迹点的距离
    double target_x_end = goal_path_.poses[idx_end].pose.position.x;
    double target_y_end = goal_path_.poses[idx_end].pose.position.y;
    double distance_end = CalculateDistance(current_pose_.pose.position.x, current_pose_.pose.position.y, target_x_end, target_y_end);
    return  distance_end;
}

//判断是否到达目标点
bool PurePursuit::isGoalReached()
{
    return goal_reached_;
}
