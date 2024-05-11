#include "icp.hpp"


clock_t start_1,end_1,start_2,end_2;
bool icp_start_flag;
std::string path;
float x_refer, y_refer, theta_refer, x, y, theta;
std::map<std::string, std::vector<std::string>> path_map;
std::map<std::string, std::vector<float>> point;

//arj21_forward
// float axis[2][6] = {{-10.0, 10.0, 20.0, 30.0, -0.6, -0.2}, 
//                     {-10.0, 10.0, 20.0, 30.0, -0.6, -0.2}};

// 滤波范围，依次是-x,x,-z,z,-y,y(右手坐标系)
float axis[2][6] = {{-3.5, 3.5, 5, 10, -0.65, 0.5}, 
                    {-3.5, 3.5, 5, 10, -0.75, 0.2}};

void SOR(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int meank, double thresh);

void PT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const uint axis, const double minLimits, const double maxLimits);

Eigen::Matrix4f ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
        const double TransformationEpsilon, const double EuclideanFitnessEpsilon, const int MaximumIterations, bool merged_flag);

float Cal_dis(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
        const double TransformationEpsilon, const double EuclideanFitnessEpsilon, const int MaximumIterations);

std::string best_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int point_size, float maxim_dis);

void merged(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 

    // 将ROS消息中的点云转换为pcl::PointCloud<pcl::PointXYZ>类型
    pcl::fromROSMsg(*cloud_msg, *cloud);
    for (size_t i = 0; i < cloud->size(); ++i) {
        float x = (*cloud)[i].x;
        float y = (*cloud)[i].y;
        float z = (*cloud)[i].z;
        (*cloud)[i].x = -x;  // 对 x 坐标值取反
    }

    // pcl::io::savePCDFileASCII("/home/dell/LiDAR/Filter_test/build/Cloud_data/TARGET.pcd", *cloud);
    // ROS_INFO("Target cloud saved.");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    visual_cloud->width = cloud->width;
    visual_cloud->height = cloud->height;
    visual_cloud->points.resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        pcl::PointXYZRGB& point = visual_cloud->points[i];
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.r = 0;  // 设置点的颜色为红色
        point.g = 0;
        point.b = 255;
    }
    pcl::io::savePCDFileASCII("/home/dell/LiDAR/Filter_test/build/Cloud_data/TARGET.pcd", *visual_cloud);
    ROS_INFO("Save Cloud Successfuly!");

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // SOR(cloud, cloud_filtered, 50, 1.0);
    // // 设置直通滤波器 
    // // x轴平行屏幕向左
    // PT(cloud_filtered, 1, axis[0][0], axis[0][1]);
    // // z轴垂直屏幕向前
    // PT(cloud_filtered, 3, axis[0][2], axis[0][3]);
    // // y轴平行屏幕向上
    // PT(cloud_filtered, 2, axis[0][4], axis[0][5]);
    // best_cloud(cloud_filtered, 1, 0.5);
    // if(path!="NO_PATH")
    // {
    //     pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud_1_filtered);
    //     ICP(cloud_filtered, cloud_1_filtered, 5, 1e-10, 0.001, 1000, 0);
    //     ROS_INFO_STREAM("ICP DONE!");
    // }
    
}

void icp_start_Callback(std_msgs::Bool icp_start_status)
{
    icp_start_flag = icp_start_status.data;
}
int main(int argc, char** argv)
{
    //如果只用一个点云进行判断，修改第一行地址即可
    // path_map["path"] = {"/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/0_0.pcd",
    //                     "/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/-0.5_0.pcd",
    //                     "/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/0.5_0.pcd",
    //                     "/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/-1.0_0.pcd",
    //                     "/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/1.0_0.pcd"};

    // //如果只用一个点云，修改第一列即可
    // point["point_x"] = {0.0f, 0.5f, -0.5f, 1.0f, -1.0f};
    // point["point_y"] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // point["point_theta"] = {0.0f ,0.0f, 0.0f, 0.0f, 0.0f};
    
    //创建ros节点 “sub_node"
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle nh;

    printf("LiDAR init!\n");
    // 订阅cloud话题，当有消息到达时触发回调函数cloudCallback
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/neuvition_cloud", 1, cloudCallback);
    // 发布姿态信息robot_pose
    // ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("/robot_pose", 1);

    // ros::Subscriber icp_start_sub = nh.subscribe<std_msgs::Bool>("/icp_start", 1, &icp_start_Callback);

    // geometry_msgs::Pose2D pose_2D;
    
    while(ros::ok())
    {
        // pose_2D.x = x;
        // pose_2D.y = y;
        // pose_2D.theta = theta;

        // pub.publish(pose_2D);
        ros::spinOnce();
    }
    return 0;
}

void SOR(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int meank, double thresh )
{

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // 设置输入点云
    sor.setInputCloud(cloud);
    // 设置平均值估计所需的点数   
    sor.setMeanK(meank);       
    // 设置标准差乘数阈值   
    sor.setStddevMulThresh(thresh); 
    // 执行滤波操作
    sor.filter(*cloud_filtered); 

}

void PT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const uint axis,const double minLimits,const double maxLimits)
{

    // 设置直通滤波器 x轴平行屏幕向左
    pcl::PassThrough<pcl::PointXYZ> pass_in;
    // 设置输入点云
    pass_in.setInputCloud(cloud);     
    // std::cout << axis << std::endl;
    switch(axis){
        case 1: pass_in.setFilterFieldName("x"); break;
        case 2: pass_in.setFilterFieldName("y"); break;
        case 3: pass_in.setFilterFieldName("z"); break;
        defult: std::cout << "axis input is wrong" << std::endl;
    }
    pass_in.setFilterLimits(minLimits, maxLimits);
    pass_in.filter(*cloud);

}

Eigen::Matrix4f ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
        const double TransformationEpsilon, const double EuclideanFitnessEpsilon, const int MaximumIterations, bool merged_flag)
{
    // 创建ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    //输入算法目标点云，即地图点云；
    icp.setInputTarget(cloud_in);

    //输入实时激光点云数据
    icp.setInputSource(cloud_out);

    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(Distance);
    icp.setTransformationEpsilon(TransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
    icp.setMaximumIterations (MaximumIterations);

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    // 获取变换矩阵
    Eigen::Matrix4f transform = icp.getFinalTransformation();

    // merged(cloud_out, cloud_in, transform);
    // 获取旋转矩阵和平移矩阵
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    // Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
    if(merged_flag)
        merged(cloud_out, cloud_in, transform);
    Eigen::Vector3f euler_angles = rotation.eulerAngles(0, 1, 2);
    // 将俯仰角设为0，表示不发生变化
    euler_angles[0] = 0;  
    // 将翻滚角设为0，表示不发生变化
    euler_angles[2] = 0;  
    rotation = (Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX()));
    transform.block<3, 3>(0, 0) = rotation;

    // std::cout << "transform matrix:" << std::endl << transform << std::endl;

    transform(1, 3) = 0.0;
    Eigen::Matrix4f transform_inv = transform.inverse();

    // 获取旋转矩阵和平移矩阵
    Eigen::Matrix3f rotation_inv = transform_inv.block<3, 3>(0, 0);
    Eigen::Vector3f euler_angles_inv = rotation_inv.eulerAngles(0, 1, 2);

    // std::cout << "transform_inv matrix:" << std::endl << transform_inv << std::endl;
    std::cout << "Yaw is:" << euler_angles_inv[1] << " Rad" << std::endl;
    std::cout << "Y is:" << transform(0, 3) << " m" << "\t";
    std::cout << "X is:" << transform(2, 3) << " m" << std::endl;

    y = transform(0, 3) + x_refer;
    x = transform(2, 3) + y_refer;
    theta = euler_angles_inv[1];

    // // 打印旋转矩阵和平移矩阵
    // std::cout << "Rotation matrix:" << std::endl << rotation << std::endl;
    // std::cout << "Translation vector:" << std::endl << translation << std::endl;


    // 获取并打印四元数
    // Eigen::Quaternionf quaternion(rotation);
    // std::cout << "Quaternion'coeffs is:" << std::endl << quaternion.coeffs() <<std::endl;
    return transform;

}

// 用于选取最佳点云，目前用不到
std::string best_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int point_size, float maxim_dis)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i<point_size; i++)
    {
        path = path_map["path"][i];
        pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud_1_filtered);
        if(Cal_dis(cloud_1_filtered,cloud, 5, 1e-10, 0.001, 1000) < maxim_dis){
            ROS_INFO_STREAM ("Meet the precision!");
            x_refer = point["point_x"][i];
            y_refer = point["point_y"][i];
            theta_refer = point["point_theta"][i];
            std::cout << "x_refer:" << x_refer << "     y_refer:" << y_refer << std::endl;
            return path;
        }

    }
    path = "NO_PATH";
    ROS_WARN("Please adjust the initial position!");
    return path;
}

float Cal_dis(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, const int Distance, 
        const double TransformationEpsilon, const double EuclideanFitnessEpsilon,const int MaximumIterations)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    //输入算法目标点云；
    icp.setInputTarget(cloud_in);

    //输入实时激光点云数据
    icp.setInputSource(cloud_out);

    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(Distance);
    icp.setTransformationEpsilon(TransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
    icp.setMaximumIterations (MaximumIterations);

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout <<" Fitness Score is: " << icp.getFitnessScore() << std::endl;
    return icp.getFitnessScore();
}


void merged(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f transform )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 将配准前后的两个点云数据合并
    origin_cloud->width = cloud_src->width + cloud_tgt->width;
    origin_cloud->height = 1;
    origin_cloud->points.resize(origin_cloud->width * origin_cloud->height);

    for (size_t i = 0; i < cloud_src->points.size(); i++)
    {
        origin_cloud->points[i].x = cloud_src->points[i].x;
        origin_cloud->points[i].y = cloud_src->points[i].y;
        origin_cloud->points[i].z = cloud_src->points[i].z;
        origin_cloud->points[i].r = 0;
        origin_cloud->points[i].g = 0;
        origin_cloud->points[i].b = 255;
    }

    int j = 0;
    for (size_t i = cloud_src->points.size(); i < origin_cloud->points.size(); i++)
    {
        origin_cloud->points[i].x = cloud_tgt->points[j].x;
        origin_cloud->points[i].y = cloud_tgt->points[j].y;
        origin_cloud->points[i].z = cloud_tgt->points[j].z;
        origin_cloud->points[i].r = 0;
        origin_cloud->points[i].g = 255;
        origin_cloud->points[i].b = 0;
        j++;
    }

    // 对合并后的点云进行ICP变换
    pcl::transformPointCloud(*cloud_src, *cloud_src, transform);

    // 将配准前后的两个点云数据合并
    merged_cloud->width = cloud_src->width + cloud_tgt->width;
    merged_cloud->height = 1;
    merged_cloud->points.resize(merged_cloud->width * merged_cloud->height);

    for (size_t i = 0; i < cloud_src->points.size(); i++)
    {
        merged_cloud->points[i].x = cloud_src->points[i].x;
        merged_cloud->points[i].y = cloud_src->points[i].y;
        merged_cloud->points[i].z = cloud_src->points[i].z;
        merged_cloud->points[i].r = 0;
        merged_cloud->points[i].g = 0;
        merged_cloud->points[i].b = 255;
    }

    j = 0;
    for (size_t i = cloud_src->points.size(); i < merged_cloud->points.size(); i++)
    {
        merged_cloud->points[i].x = cloud_tgt->points[j].x;
        merged_cloud->points[i].y = cloud_tgt->points[j].y;
        merged_cloud->points[i].z = cloud_tgt->points[j].z;
        merged_cloud->points[i].r = 0;
        merged_cloud->points[i].g = 255;
        merged_cloud->points[i].b = 0;
        j++;
    }

    pcl::io::savePCDFileASCII("/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/merged.pcd", *merged_cloud);
    pcl::io::savePCDFileASCII("/home/dell/catkin_ws/src/Lidar/src/Point_filter_Cloud/origin.pcd", *origin_cloud);
}
