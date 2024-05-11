#include "ros/ros.h" 
#include "std_msgs/String.h" 
#include <neuvition_driver/test.h> 
#include <sstream> 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <math.h>

#include <string>
#include <iostream>

#include <boost/version.hpp>
#include <boost/timer.hpp>

#include "handler_allocator.hpp"
#include "neuv_defs.hpp"
#include "driver.h"
#include <tf/transform_listener.h>
#include <pcl/point_types_conversion.h>    //pcl_conversions.h
#include <sensor_msgs/Imu.h>

#include <stdlib.h>
#include <stdint.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//#include "matrix_quarternion.hpp"

using namespace std;
using namespace pcl;


#define UNUSED(var) (void)(var)

int isImageRotate = 0;

int isTimemode = 1;
neuvition::NeuvUnits predata;

uint32_t iFrontFrameTime = 0;
uint32_t iNowFrameTime = 0;
uint32_t iMsecTime = 0 ;

/* YELLOW ff/ff/00 RED ff/00/00 MAGENTA ff/00/ff BLUE 00/00/ff CYAN 00/ff/ff GREEN 00/ff/00 */
const uint32_t coloredge[6]={ 0xffff00, 0xff0000, 0xff00ff, 0x0000ff, 0x00ffff, 0x00ff00 };

uint32_t tof_cycle=65000;  // orig:9000 2m:13000 10m:65000 30m:200000

double get_timestamp(void) 
{
    struct timeval now;
    gettimeofday(&now, 0);
    
    return (double)(now.tv_sec) + (double)(now.tv_usec)/1000000.0;
}

void showretval(int ret)
{ 
    if (ret==0)
        return;
    std::cout<< "ret:"<< ret << std::endl;
}
    
namespace neuvition_driver 
{

    class gEventHandler : public neuvition::INeuvEvent
    {

    private:
        neuvitionDriver *neudrv;

    public:
        gEventHandler(neuvitionDriver* _drv):neudrv(_drv)
        {
            std::cout << "The callback instance of driver initialized" << std::endl;
        }

        virtual void on_connect(int code, const char* msg) 
        {
            std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvtion: on_connect: " << code << "(" << msg << ")" << std::endl;
            if (code==0)
            {
                int ret = neuvition::set_reconnect_params(false, 5); showretval(ret);
            }
        }

        virtual void on_disconnect(int code) 
        {
            std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: on_disconnect: "<< code << std::endl;
        }

        virtual void on_lidar_info_status(neuvition::LidarInfoStatus*)
		{
			//TODO: nothing		
		}

        virtual void on_response(int code, enum neuvition::neuv_cmd_code cmd) 
        {
            std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: on_response[ " << cmd << " ]: " << code << std::endl;
            if (cmd==neuvition::NEUV_CMD_START_SCAN && code==0 )    { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: scanning started..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_STOP_SCAN && code==0 )     { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: scanning stopped..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_START_STREAM && code==0 )  { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: streaming started..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_STOP_STREAM && code==0 )   { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: streaming stopped..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_GET_PARAMS && code==0 )    { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: device parameter is  synced..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_GPS_UPDATED && code==0 )    { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: gps info message..." << std::endl; }
        }

        virtual void on_framedata(int, int64_t, const neuvition::NeuvUnits&, const neuvition::nvid_t&,const neuvition::NeuvWireDatas&){}

        virtual void on_framedata(int, int64_t, const neuvition::NeuvUnits&, const neuvition::nvid_t&,const neuvition::NeuvCt&){}

        virtual void on_framedata(int code,int64_t microsec, const neuvition::NeuvUnits& data, const neuvition::nvid_t& frame_id) 
        {
               std::cout<< "on_framedata -- TS: " << get_timestamp()  << ", -- code: " << code << "point size = " << data.size() <<  endl;
      
            PointCloudT cloud_;
            cloud_.reserve(data.size());

            predata = data;

            int i = 0 ;
		neudrv->color_mode = 1;
            for (neuvition::NeuvUnits::const_iterator iter = data.begin(); iter != data.end(); iter++) 
            {
                const neuvition::NeuvUnit& np = (*iter);
                PointT point;

                //PointXYZRGBATI
                point.x = np.x*0.001; point.y = np.y*0.001; point.z = np.z*0.001;

                if(np.z==0) 
                    continue;

                point.a = 255;
                  point.intensity = np.intensity;
           
            point.time_sec = np.time_sec;
                point.time_usec = np.time_usec;
                
                if (i == 0)
                {
                    printf( "timestamp sec = %ld, usec=%d", np.time_sec, np.time_usec);
                    iNowFrameTime = point.time_sec;
                }

                i++;
#if 1

                if (neudrv->color_mode==0) /* 0:polychrome */
                { 
                    uint32_t tofcolor=np.tof%tof_cycle;
                    uint32_t tofclass=tofcolor/(tof_cycle/6);
                    uint32_t tofreman=tofcolor%(tof_cycle/6);
                    uint32_t cbase=coloredge[(tofclass+0)%6];
                    uint32_t cnext=coloredge[(tofclass+1)%6];
                    uint32_t ccand=cnext&cbase; UNUSED(ccand);
                    uint32_t ccxor=cnext^cbase;
                    int shift=__builtin_ffs(ccxor)-1;
                    int xincr=(cnext>cbase) ? 1 : -1;
                    uint32_t crender=cbase+xincr*((int)(tofreman*(256.0/(tof_cycle/6)))<<shift);
                    point.r = (crender&0xff0000)>>16;
                    point.g = (crender&0x00ff00)>>8;
                    point.b = (crender&0x0000ff)>>0;
                } 
                else if (neudrv->color_mode==1) /* 1:camera */
                { 
                    point.r = np.r; point.g = np.g; point.b = np.b;
                } 
                else if (neudrv->color_mode==2) /* 2:intensity */
                { 
                    //std::cout << "intensity" << endl;
                    uint32_t tofcolor=np.intensity;
                    uint32_t tofclass=tofcolor/(256/6);
                    uint32_t tofreman=tofcolor%(256/6);
                    uint32_t cbase=coloredge[(tofclass+0)%6];
                    uint32_t cnext=coloredge[(tofclass+1)%6];
                    uint32_t ccand=cnext&cbase; UNUSED(ccand);
                    uint32_t ccxor=cnext^cbase;
                    int shift=__builtin_ffs(ccxor)-1;
                    int xincr=(cnext>cbase) ? 1 : -1;
                    uint32_t crender=cbase+xincr*((int)(tofreman*(256.0/(256/6)))<<shift);
                    point.r = (crender&0xff0000)>>16;
                    point.g = (crender&0x00ff00)>>8;
                    point.b = (crender&0x0000ff)>>0;
                }
#endif 

                cloud_.push_back(point);  

            }// for (neuvition::NeuvUnits::const_iterator iter = data.begin(); iter != data.end(); iter++) 

        
            if (iFrontFrameTime  == 0 )
            {
                iMsecTime = 0 ;
            }
            else
            {
                iMsecTime += ((iNowFrameTime - iFrontFrameTime) / 1000);
            }

            if (iMsecTime > 1000 )
            {
                iMsecTime -= 1000 ;
            }
       
               printf("iMsecTime = %ld\n",iMsecTime);

           //neuvition::jason_camearinfo_clear(frame_id);
	   /*
	   Rotating point cloud
           */

           iFrontFrameTime = iNowFrameTime;
	   float FPi = 3.1415926;
    	   float newthetax = 0 * (FPi / 180.0); // 0 Angle .You can enter any Angle you want
	   Eigen::Affine3f transform_X = Eigen::Affine3f::Identity();
    	   transform_X.translation() << 0,0,0; // translation
           transform_X.rotate(Eigen::AngleAxisf(newthetax, Eigen::Vector3f::UnitX())); // Rotate around X
           pcl::transformPointCloud(cloud_, cloud_, transform_X);
           neudrv->neuProcessPoint(cloud_);
       }

       virtual void on_imudata(int code,int64_t microsec,const neuvition::NeuvUnits& data,const neuvition::ImuData& imu) 
       {
            sensor_msgs::Imu imu_data;

            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "neuvition";

#define QP(n) (1.0f/(1<<n))

            imu_data.orientation.x = QP(14) * imu.quat_i;
            imu_data.orientation.y = QP(14) * imu.quat_j;
            imu_data.orientation.z = QP(14) * imu.quat_k;
            imu_data.orientation.w = QP(14) * imu.quat_r;
            
            imu_data.linear_acceleration.x = 0.0f; 
            imu_data.linear_acceleration.y = 0.0f;
            imu_data.linear_acceleration.z = 0.0f;

            imu_data.angular_velocity.x = 0.0f; 
            imu_data.angular_velocity.y = 0.0f; 
            imu_data.angular_velocity.z = 0.0f;

            neudrv->IMU_pub.publish(imu_data);
       }

    virtual void on_pczdata(bool status) 
    {

    }
    virtual void on_Ladar_Camera(neuvition::NeuvCameraLadarDatas * neuvcameraladarpos)// neuvition::NeuvCameraLadarDatas 数据容器，通过循环遍历取出里面的单个元素
    {

    }
virtual void on_framestart1(int nCode) {}
virtual void on_framestart2(int nCode) {}

       virtual void on_mjpgdata(int code, int64_t microsec, cv::Mat Mat) 
    { 
        if (Mat.empty()) 
            return;
    if(0)
{	
	  transpose(Mat,Mat);
            flip(Mat,Mat,1);
            transpose(Mat,Mat);
            flip(Mat,Mat,1);
}
else
{
}

        sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Mat).toImageMsg();

        neudrv->neuProcessImage(msg_);
    }
};

neuvitionDriver::neuvitionDriver(ros::NodeHandle node, ros::NodeHandle private_nh) 
{
    ROS_INFO_STREAM(" ros version  1.0.2");

    // use private node handle to get parameters
    private_nh.param<std::string>("frame_id", frame_id, std::string("neuvition"));
    ROS_INFO_STREAM("frameID [ " << frame_id << " ]" );

    private_nh.param<int>("pwm_value", pwm_value, 55);
    ROS_INFO_STREAM("pwmValue [ "<< pwm_value << " ] %");

    private_nh.param<int>("laser_period", laser_period, 0);
    ROS_INFO_STREAM("laserPeriod [ "<< laser_period << " ]");

    private_nh.param<int>("data_frame", data_frame, 0);
    ROS_INFO_STREAM("dataFrame [ "<< data_frame << " ]");

    private_nh.param<int>("color_mode", color_mode, 0);
    ROS_INFO_STREAM("color_mode [ "<< color_mode << " ]");


    private_nh.param<int>("upstream_port", upstream_port, 6668);
    ROS_INFO_STREAM("upstreamPort [ "<< upstream_port << " ]");

    private_nh.param<std::string>("upstream_host", upstream_host, std::string("192.168.12.101"));
    ROS_INFO_STREAM("upstreamHost [ "<< upstream_host << " ]");


    private_nh.param<bool>("video_fusion", video_fusion, true);
    ROS_INFO_STREAM("video_fusion [ "<< video_fusion << " ]");  

    private_nh.param<int>("laser_image", isImageRotate, 0);
    ROS_INFO_STREAM("laser_image [ "<< isImageRotate << " ]");

    int ifilterSt;
    private_nh.param<int>("laser_filters", ifilterSt, 0);
    ROS_INFO_STREAM("laser_filters [ "<< ifilterSt << " ]");

    private_nh.param<int>("laser_time", isTimemode, 0);
    ROS_INFO_STREAM("laser_time [ "<< isTimemode << " ]");

    neuvition::set_g_filter_enabled(ifilterSt);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<neuvition_driver::NeuvitionNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<neuvition_driver::NeuvitionNodeConfig>::CallbackType f;
    f = boost::bind (&neuvitionDriver::neuCallback, this, _1, _2);
    srv_->setCallback (f); // Set callback function und call initially 

    output_cloud_ = node.advertise<sensor_msgs::PointCloud2>("neuvition_cloud", 1);
    image_transport::ImageTransport it_(node);
    output_img_ = it_.advertise("neuvition_image", 1);

    output_cloud_camera = node.advertise<neuvition_driver::test>("neuvition_driver",20);

    //advertise IMU message publisher
    IMU_pub = node.advertise<sensor_msgs::Imu>("neuvition_IMU",20);;
}

void neuvitionDriver::neuCallback(neuvition_driver::NeuvitionNodeConfig &config,  uint32_t level) 
{
    std::cout << "neuvitionDriver::neuCallback" << endl;
    std::cout << "config::pwm_value" << config.pwm_value << endl;
     std::cout << "pwm_value" << pwm_value << endl;

    if(pwm_value != config.pwm_value) {
        pwm_value = config.pwm_value;
        neuSetPwm(50);
    }

    if(laser_period != config.laser_period) {
        laser_period = config.laser_period;
        neuSetLaserPeriod(laser_period);
    }

    if (data_frame != config.data_frame) {
        std::cout << data_frame << "11111111111" << config.data_frame << endl;
        data_frame = config.data_frame;
        neuSetDataFrame(data_frame);
    }

    if (color_mode != config.color_mode) {
        color_mode = config.color_mode;
        neuSetColorMode(color_mode);
    }


    if (video_fusion != config.video_fusion) {
      video_fusion = config.video_fusion;
      neuStopData();
      while(neuvition::is_streaming()) {}
        neuVideoFusion(video_fusion);
      neuStartData();
    }

    isImageRotate = config.laser_image;
    
  

  }

void neuvitionDriver::neuConnect() 
{
    printf ("Start connecting ...\n");
    neuvition::set_camera_status(true);
    neuvition::set_flip_axis(false, true);
    neuvition::set_mjpg_curl(true);

    neuvition::INeuvEvent* phandler = new gEventHandler(this);

    int ret=neuvition::setup_client(upstream_host.c_str(), upstream_port, phandler, false);

    UNUSED(ret);
    usleep(2000000);
}


void neuvitionDriver::neuStartScan() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..start scan"<<std::endl;
    int ret=neuvition::start_scan(); showretval(ret); // ask device to start scanning
    usleep(200000);
}

void neuvitionDriver::neuStartData() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..start stream"<<std::endl;
    int ret=neuvition::start_stream(); showretval(ret); // ask device to start streaming
    usleep(200000);

}


void neuvitionDriver::neuStopScan() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..stop scan"<<std::endl;
    int ret=neuvition::stop_scan(); showretval(ret); // ask device to stop scanning
    usleep(200000);
}


void neuvitionDriver::neuStopData() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..stop stream"<<std::endl;
    int ret=neuvition::stop_stream(); showretval(ret); // ask device to stop streaming
    usleep(200000);

}

void neuvitionDriver::neuDisconnect() {

    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..teardown client"<<std::endl;
    int ret=neuvition::teardown_client(); showretval(ret);
    sleep(1);

}

void neuvitionDriver::neuSetLaserPeriod(int value) {

   
    printf ("Laser Period: [%d]\n", value);
   
    int ret=neuvition::set_laser_interval(value);//3
    UNUSED(ret);
    usleep(2000000);
}


void neuvitionDriver::neuSetDataFrame(int value) {

    //0:30fps  1:20fps  2:15fps  3:10fps  4:6fps  5:5fps  6:3fps  7:2fps  8:1fps
    std::cout << "value = " << value ;
    int fps=0;
    int sramreads=0;
    switch(value) {
        case(0): { fps = 5; break;}
        case(1): { fps = 6; break;}
        case(2): { fps = 10; break;}
        case(3): { fps = 15; break;}
        case(4): { fps = 30; break;}
        case(5): { fps = 1; break;}
    }
    printf ("Data Frame: [%d]fps\n", fps);
    int ret = neuvition::set_frame_frequency(fps);
       UNUSED(ret);
    usleep(2000000);
    std::cout << "frame"<< neuvition::get_frame_frequency() <<endl;

}


void neuvitionDriver::neuSetPwm(int value) {
    //<=65%
    printf("PWM Value: [%d]%\n", value);
    int ret = neuvition::set_laser_power(value);
    UNUSED(ret);
    usleep(2000000);

}


void neuvitionDriver::neuSetColorMode(int value) {
    // 0:PolyChrome  1:Camera  2:Intensity
    color_mode = value;
    std::string mode;
    switch(value) {
        case 0: { mode="PolyChrome"; break; }
        case 1: { mode="Camera"; break; }
    }
    printf("Color Mode:[%s]\n", mode.c_str());

}

void neuvitionDriver::neuVideoFusion(bool value) {

    printf("Camera: [%s]\n",true?"Open":"Close");
    int ret  = neuvition::set_camera_status(true);
    ret=neuvition::set_mjpg_curl(true); 
    UNUSED(ret);
    usleep(200000);
}


void neuvitionDriver::neuSet_g_filter_enabled(bool value)
{
    printf("neuSet_g_filter_enabled: [%s]\n",value? "True":"False");
    int ret  = neuvition::set_g_filter_enabled(value);
    usleep(200000);
}

void neuvitionDriver::neuSet_g_gaus_fit_enabled(bool value)
{
    printf("set_g_gaus_fit_enabled: [%s]\n",value? "True":"False");
    int ret  = neuvition::set_g_gaus_fit_enabled(value);
    usleep(200000);
}

void neuvitionDriver::neuSet_g_linear_fit_enabled(bool value)
{
    printf("set_g_linear_fit_enabled: [%s]\n",value? "True":"False");
    int ret  = neuvition::set_g_linear_fit_enabled(value);
    usleep(200000);
}

void neuvitionDriver::neuInit() 
{

    neuConnect();

    usleep(200000);

    if(neuvition::is_connected()) 
    {
        std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..connected"<<std::endl;

        double hfov=neuvition::get_hfov();
        double vfov=neuvition::get_vfov();
        int device_type=neuvition::get_device_type();
	
	neuvition::set_flip_axis(true,true);
        //neuvition::set_npvt_value(3);

        /*neuvition::NeuPosCor pos_cor=neuvition::get_poscor_params();
        double bias_y=0.0;
        neuvition::compute_tof2xyz_table(hfov, vfov, bias_y, device_type, pos_cor); 

        std::cout<< "HFov="<< hfov << ", VFov="<<vfov << std::endl;
        neuvition::LaserIncidentAngles laserangles=neuvition::get_laser_angles(); 
        std::cout<< "LaserIncidentAngles: " << laserangles[0] << "," << laserangles[1] << std::endl;
        neuvition::compute_tof2xyz_table_with_multi_lasers(laserangles, device_type, pos_cor);*/



        neuStartScan();
        if(neuvition::is_scanning())  std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..scanning" <<std::endl;

        neuSetPwm(100);
        //0: 340HZ   1: 500KHZ    2: 750MHZ   3: 1MHZ
        neuSetLaserPeriod(3);
        //0:30fps  1:20fps  2:15fps  3:10fps  4:6fps  5:5fps  6:3fps  7:2fps  8:1fps
        neuSetDataFrame(5);
        neuSetColorMode(color_mode);
        neuVideoFusion(video_fusion);
	

        neuStartData();
        usleep(200000);

        neuvition::set_gps_status(true);
        if(neuvition::is_streaming())
        { 
            std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..streaming"<<std::endl;
        }

        sleep(1);

        //enable IMU data
        //neuvition::set_imu_status(true);  //can't work with low version of ARM firmware

        sleep(1);

    } 
    else 
    {
        std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" failed to connect .. "<<std::endl;
        exit(1);
    }

}


void neuvitionDriver::neuProcessPoint(PointCloudT &cloud) 
{
      //need convert to pcl::PointXYZI, 
    //otherwise the ROS_subscriber will popup warning 'Failed to find match for field intensity'
    pcl::PointCloud<PointT>   pcl_points;
    pcl_points.resize(cloud.size());

    for (int i = 0; i < cloud.size(); i++) 
    {
        pcl_points.at(i).x = cloud.at(i).x;
        pcl_points.at(i).y = cloud.at(i).y;
        pcl_points.at(i).z = cloud.at(i).z;
	pcl_points.at(i).r = cloud.at(i).r;
	pcl_points.at(i).g = cloud.at(i).g;
	pcl_points.at(i).b = cloud.at(i).b;
	pcl_points.at(i).a = 255;
	pcl_points.at(i).time_sec = cloud.at(i).time_sec;
        pcl_points.at(i).intensity = cloud.at(i).intensity;
    }
    
    pcl::toROSMsg(pcl_points, neuvition_pub); 

    neuvition_pub.header.stamp = ros::Time::now();
    neuvition_pub.header.frame_id = frame_id;

    output_cloud_.publish(neuvition_pub);
}


void neuvitionDriver::neuProcessImage(sensor_msgs::ImagePtr &msg) {

     msg->header.stamp = ros::Time::now();
    output_img_.publish(msg);


}
void neuvitionDriver::neuProcessCameraLadar(  std::vector<long int>  vcameraladarpos )
{
    neuvition_driver::test msg;
              msg.data = vcameraladarpos;
        output_cloud_camera.publish(msg);
    //vcameraladarpos.clear();
}


}////end of file



