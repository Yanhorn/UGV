#include <iostream>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  if(msg->x||msg->y){
  ROS_INFO("Received Pose2D: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
  }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_node");
    ros::NodeHandle nh;
    
    printf("Pose init!\n");

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose2D>("robot_pose", 10, &poseCallback);
    while(ros::ok())
    {
        ros::spinOnce();
	}
    return 0;
}
