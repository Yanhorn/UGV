#ifndef NEUVITION_DRIVER_H
#define NEUVITION_DRIVER_H
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "neuvition_driver/NeuvitionNodeConfig.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


// Define PointType 
struct PointXYZRGBATI              //定义点类型结构
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_UNION_RGB;
     uint32_t time_sec;
        uint32_t time_usec;
uint32_t intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作
}EIGEN_ALIGN16;// 强制SSE对齐


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBATI,// 注册点类型宏
    (float,x,x)
    (float,y,y)
    (float,z,z)
    (uint32_t,rgba,rgba)
  (uint32_t, time_sec, time_sec)
(uint32_t, time_usec, time_usec)
    (uint32_t,intensity,intensity)
    )
    

typedef PointXYZRGBATI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace neuvition_driver {
    class neuvitionDriver
    {

    public:
        neuvitionDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~neuvitionDriver() {}


        //Callback for dynamic reconfigure
        void neuCallback(neuvition_driver::NeuvitionNodeConfig &config,
          uint32_t level);

        //Pointer to dynamic reconfigure service srv_
        boost::shared_ptr<dynamic_reconfigure::Server<neuvition_driver::
        NeuvitionNodeConfig> > srv_;

        ros::Publisher output_cloud_;
        sensor_msgs::PointCloud2 neuvition_pub;

        //IMU publisher 
        ros::Publisher IMU_pub;

        ros::Publisher output_cloud_camera;
        image_transport::Publisher output_img_;

        // configuration parameters
        std::string frame_id;            
        int data_mode;                   
        int pwm_value;
        int laser_period;
        int data_frame;
        int upstream_port;
        std::string upstream_host;
        int color_mode;
        bool video_fusion;

        void neuConnect();
        void neuStartScan();
        void neuStartData();
        void neuStopScan();
        void neuStopData();
        void neuDisconnect();
        void neuSetLaserPeriod(int value);

        void neuSet_g_filter_enabled(bool enabled);
        void neuSet_g_gaus_fit_enabled(bool enabled);
        void neuSet_g_linear_fit_enabled(bool value);

        void neuSetPwm(int value);
        void neuSetDataFrame(int value);
        void neuSetColorMode(int value);
        void neuVideoFusion(bool value);
        void neuInit();
        void neuProcessPoint(PointCloudT &cloud);
        void neuProcessCameraLadar(  std::vector<long int>  vcameraladarpos);

        //image
        void neuProcessImage(sensor_msgs::ImagePtr &msg);



    };

}


double get_timestamp(void);
void showretval(int ret);



#endif
