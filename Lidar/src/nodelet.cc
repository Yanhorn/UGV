/** \file
 *
 *  ROS driver nodelet for the Neuvition 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver.h"

namespace neuvition_driver
{

class NeuvdrvNodelet: public nodelet::Nodelet
{
public:

  NeuvdrvNodelet():
    running_(false)
  {}


  ~NeuvdrvNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
      }
  }

private:

  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;               ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<neuvitionDriver> drv_; ///< driver implementation class
};

void NeuvdrvNodelet::onInit()
{
  // start the driver
  drv_.reset(new neuvitionDriver(getNodeHandle(), getPrivateNodeHandle()));


  // spawn device poll thread
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&NeuvdrvNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void NeuvdrvNodelet::devicePoll()
{
  
  drv_->neuInit();
  running_ = true;
  while(ros::ok())
    {
      // poll device until end of file
      // running_ = drv_->poll();
      // if (!running_) break;
      ros::spinOnce();
    }
  running_ = false;
  drv_->neuStopData();
  drv_->neuStopScan();
  drv_->neuDisconnect();
  
}

} // namespace neuvition_driver

// Register this plugin with pluginlib.  Names must match nodelet_neuvition.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(neuvition_driver::NeuvdrvNodelet, nodelet::Nodelet)
