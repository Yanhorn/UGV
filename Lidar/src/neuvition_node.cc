#include <ros/ros.h>
#include "driver.h"
#include "neuv_defs.hpp"
#include <sensor_msgs/PointCloud2.h>




int main(int argc, char** argv)
{
    ros::init(argc, argv, "neuvition_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~config");

    // start the driver
    neuvition_driver::neuvitionDriver neu(node, private_nh);


    neu.neuInit();


    // loop until shut down or end of file))
   // while(1);
    while(ros::ok() )
    {
        ros::spinOnce();
         usleep(200000);
    }

    
    neu.neuStopData();
    neu.neuStopScan();
    neu.neuDisconnect();
    return 0;
}
