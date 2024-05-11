#include <iostream>
#include <map>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include <ctime>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/duration.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/distances.h>