#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include<iomanip>//必要头文件

// save map
#include "livox_color/save_map.h"
#include "livox_color/save_pose.h"

#include <thread>
#include <mutex>
#include <unistd.h>

using namespace std ;

#define Hmax 720
#define Wmax 1280
#define H Hmax
#define W Wmax

// 存储数据队列
std::mutex buff_mutex_;

std::deque<sensor_msgs::PointCloud2ConstPtr>  lidar_buff_ ;
std::deque<sensor_msgs::ImageConstPtr>  image_buff_ ;
std::deque<nav_msgs::Odometry::ConstPtr>  odom_buff_ ;

std::deque<sensor_msgs::ImageConstPtr>  unsyced_image_ ;
std::deque<nav_msgs::Odometry::ConstPtr>  unsyced_odom_ ;


Eigen::Isometry3d  Twlidar  ;			//  FASTLIO2 订阅当前里程计
vector<double>       extrinT(16, 0.0);				//  lidar2camera
vector<double>       intrisicT(9, 0.0);				//   camera  intrinT
vector<double>       ditortion(5, 0.0);				//  畸变系数

/*ROS param*/
string  lidar_topic, lidar_color_topic, camera_topic, odom_topic, frame_id ;

/*ROS  NODE*/
ros::Subscriber subOdom ;   // 订阅FASTLIO2里程计
ros::Subscriber subImage ;
//image_transport::Subscriber subImage ;   // 订阅图像
ros::Subscriber subCloud ;  // 订阅点云
ros::Publisher pubCloud ;   // 发布彩色点云

/* 自定义的PointXYZRGBIL（pcl没有PointXYZRGBIL、PointXYZRGBI结构体）*/
struct PointXYZRGBIL
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	uint32_t label;
	PCL_ADD_INTENSITY;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
	PointXYZRGBIL,
	(float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(uint32_t, label, label)(float, intensity, intensity))

typedef PointXYZRGBIL PointType;
// 使用pcl::PointXYZRGBNormal结构体代替自定义的PointXYZRGBI，为了livox_color_mapping建图，自定义结构体不支持很多内置函数
// typedef pcl::PointXYZRGBNormal PointType;

// saveMap
ros::ServiceServer srvSaveMap;
string savePCDDirectory;    // 保存路径
vector<pcl::PointCloud<PointType>::Ptr> mapCloudKeyFrames;   // 历史所有关键帧的平面点集合（降采样）

//全局变量都能访问，图像回调中写，点云回调中读
cv::Vec3b image_color[H][W];

// 内外参
cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);	   // 畸变向量
cv::Mat intrisic = cv::Mat::eye(3, 3, CV_64F);			   // 内参3*3矩阵
cv::Mat intrisicMat(3, 4, cv::DataType<double>::type);	   // 内参3*4的投影矩阵，最后一列是三个零
cv::Mat extrinsicMat_RT(4, 4, cv::DataType<double>::type); // 外参旋转矩阵3*3和平移向量3*1
void CalibrationData(void)
{
	// extrinsic
	//  0.0144457   -0.999828  0.0116559  -0.115962
	// -0.0142691  -0.0118621  -0.999828 -0.0215207
	// 0.999794  0.0142769  -0.014438  -0.0131816
	// 0  0  0  1
	extrinsicMat_RT.at<double>(0, 0) = extrinT[0];
	extrinsicMat_RT.at<double>(0, 1) = extrinT[1];
	extrinsicMat_RT.at<double>(0, 2) = extrinT[2];
	extrinsicMat_RT.at<double>(0, 3) =  extrinT[3];

	extrinsicMat_RT.at<double>(1, 0) = extrinT[4];
	extrinsicMat_RT.at<double>(1, 1) =extrinT[5];
	extrinsicMat_RT.at<double>(1, 2) = extrinT[6];
	extrinsicMat_RT.at<double>(1, 3) =extrinT[7];

	extrinsicMat_RT.at<double>(2, 0) = extrinT[8];
	extrinsicMat_RT.at<double>(2, 1) = extrinT[9];
	extrinsicMat_RT.at<double>(2, 2) = extrinT[10];
	extrinsicMat_RT.at<double>(2, 3) = extrinT[11];

	extrinsicMat_RT.at<double>(3, 0) = extrinT[12];
	extrinsicMat_RT.at<double>(3, 1) = extrinT[13];
	extrinsicMat_RT.at<double>(3, 2) = extrinT[14];
	extrinsicMat_RT.at<double>(3, 3) = extrinT[15];

	// intrinsic
	// 913.197692871094  0  648.626220703125;
	//  0  913.52783203125  358.518096923828;
	//  0  0  1
	// ditortion
	// -0.008326874784366894 -0.06967846599874981 0.006185220615585947 -0.01133018681519818  0.5462976722456516
	intrisicMat.at<double>(0, 0) = intrisic.at<double>(0, 0) = intrisicT[0];
	intrisicMat.at<double>(0, 1) = 0.000000e+00;
	intrisicMat.at<double>(0, 2) = intrisic.at<double>(0, 2) = intrisicT[2];
	intrisicMat.at<double>(0, 3) = 0.000000e+00;
	intrisicMat.at<double>(1, 0) = 0.000000e+00;
	intrisicMat.at<double>(1, 1) = intrisic.at<double>(1, 1) = intrisicT[4];
	intrisicMat.at<double>(1, 2) = intrisic.at<double>(1, 2) = intrisicT[5];
	intrisicMat.at<double>(1, 3) = 0.000000e+00;
	intrisicMat.at<double>(2, 0) = 0.000000e+00;
	intrisicMat.at<double>(2, 1) = 0.000000e+00;
	intrisicMat.at<double>(2, 2) = 1.000000e+00;
	intrisicMat.at<double>(2, 3) = 0.000000e+00;
	distCoeffs.at<double>(0) = ditortion[0];
	distCoeffs.at<double>(1) = ditortion[1];
	distCoeffs.at<double>(2) = ditortion[2];
	distCoeffs.at<double>(3) = ditortion[3];
	distCoeffs.at<double>(4) = ditortion[4];
}

//点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    buff_mutex_.lock();
    lidar_buff_.push_back(laserCloudMsg);
    buff_mutex_.unlock();
}

void imageCallback(const sensor_msgs::ImageConstPtr &ImageMsg)
{
    buff_mutex_.lock();
    unsyced_image_.push_back(ImageMsg);
    buff_mutex_.unlock();
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &OdomMsg)
{
    buff_mutex_.lock();
    unsyced_odom_.push_back(OdomMsg);
    buff_mutex_.unlock();
}

void ColorPointcloud()
{
    sensor_msgs::PointCloud2ConstPtr  laserCloudMsg = lidar_buff_.front();
    sensor_msgs::ImageConstPtr ImageMsg = image_buff_.front();
    nav_msgs::Odometry::ConstPtr OdomMsg = odom_buff_.front();

    /*image process*/
    try
    {
        cv::Mat image = cv_bridge::toCvShare(ImageMsg, "bgr8")->image; //image_raw就是我们得到的图像了
        // 去畸变，可选
        cv::Mat map1, map2;
        cv::Size imageSize = image.size();
        cv::initUndistortRectifyMap(intrisic, distCoeffs, cv::Mat(), cv::getOptimalNewCameraMatrix(intrisic, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
        cv::remap(image, image, map1, map2, cv::INTER_LINEAR); // correct the distortion
        // cv::imwrite("/home/lory/1.bmp",image);				//   保存看畸变矫正后的结果
        for (int row = 0; row < H; row++)
        {
            for (int col = 0; col < W; col++)
            {
                image_color[row][col] = (cv::Vec3b)image.at<cv::Vec3b>(row, col);
            }
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not conveextrinsicMat_RT from '%s' to 'bgr8'.", ImageMsg->encoding.c_str());
    }

    /*odom process*/
     Eigen::Vector3d t_w_lidar(OdomMsg->pose.pose.position.x, OdomMsg->pose.pose.position.y, OdomMsg->pose.pose.position.z);	 //    当前载体在世界系下的位置
     Eigen::Quaterniond q_w_lidar(OdomMsg->pose.pose.orientation.w, OdomMsg->pose.pose.orientation.x, OdomMsg->pose.pose.orientation.y, OdomMsg->pose.pose.orientation.z);   //    当前载体在世界系下的姿态
     q_w_lidar.normalize();
     Eigen::Isometry3d T_w_lidar(q_w_lidar);
     T_w_lidar.pretranslate(t_w_lidar);
     Twlidar = T_w_lidar ;

     /*pointcloud process*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); //livox点云消息包含xyz和intensity
    pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr);										   //把msg消息指针转化为PCL点云
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    pcl::PointCloud<PointType>::Ptr fusion_pcl_ptr(new pcl::PointCloud<PointType>); //放在这里是因为，每次都需要重新初始化

    for (int i = 0; i < raw_pcl_ptr->points.size(); i++)
    {
        Eigen::Vector3d x_w(raw_pcl_ptr->points[i].x, raw_pcl_ptr->points[i].y, raw_pcl_ptr->points[i].z);
        Eigen::Vector3d x_lidar =  Twlidar.inverse()  *  x_w   ;
        X.at<double>(0, 0) = x_lidar.x();
        X.at<double>(1, 0) = x_lidar.y();
        X.at<double>(2, 0) = x_lidar.z();
        X.at<double>(3, 0) = 1;
        Y = intrisicMat * extrinsicMat_RT * X; //雷达坐标转换到相机坐标，相机坐标投影到像素坐标
        cv::Point pt;						   // (x,y) 像素坐标
        // Y是3*1向量，pt.x是Y的第一个值除以第三个值，pt.y是Y的第二个值除以第三个值，为什么是下面这种写法？？
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
        if (pt.x >= 0 && pt.x < W && pt.y >= 0 && pt.y < H && x_lidar.x() > 0) //&& raw_pcl_ptr->points[i].x>0去掉图像后方的点云
        {
            PointType p;
            p.x = raw_pcl_ptr->points[i].x;
            p.y = raw_pcl_ptr->points[i].y;
            p.z = raw_pcl_ptr->points[i].z;
            //点云颜色由图像上对应点确定
            p.b = image_color[pt.y][pt.x][0];
            p.g = image_color[pt.y][pt.x][1];
            p.r = image_color[pt.y][pt.x][2];
            p.intensity = raw_pcl_ptr->points[i].intensity; //继承之前点云的intensity
            fusion_pcl_ptr->points.push_back(p);
        }
    }
    fusion_pcl_ptr->width = fusion_pcl_ptr->points.size();
    fusion_pcl_ptr->height = 1;

    sensor_msgs::PointCloud2 fusion_msg;
    pcl::toROSMsg(*fusion_pcl_ptr, fusion_msg);			   //将点云转化为消息才能发布
    fusion_msg.header.frame_id = frame_id ;			   //   Fastlio2 中 frame-id 为 camera_init
    fusion_msg.header.stamp = laserCloudMsg->header.stamp; // 时间戳和/livox/lidar 一致
    pubCloud.publish(fusion_msg);						   //发布调整之后的点云数据

    /*SaveMap*/
    mapCloudKeyFrames.push_back(fusion_pcl_ptr);		// 存储历史帧点云

    lidar_buff_.pop_front();
    image_buff_.pop_front();
    odom_buff_.pop_front();
}



/**
 * 保存全局关键帧特征点集合
*/
bool saveMapService(livox_color::save_mapRequest& req, livox_color::save_mapResponse& res)
{
      string saveMapDirectory;
    
      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      if(req.destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
      else saveMapDirectory = std::getenv("HOME") + req.destination;
      cout << "Save destination: " << saveMapDirectory << endl;

      // 提取历史关键帧角点、平面点集合
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

      for (int i = 0; i < (int)mapCloudKeyFrames.size(); i++) {
			*globalMapCloud   += *mapCloudKeyFrames[i] ;
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << mapCloudKeyFrames.size() << " ...";
      }

      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);       //  稠密地图
      res.success = ret == 0;

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      return true;
}


bool ImageSyncData(std::deque<sensor_msgs::ImageConstPtr> &UnsyncedData,std::deque<sensor_msgs::ImageConstPtr> &SyncedData, double sync_time){
    while(UnsyncedData.size() >= 1){
        if(UnsyncedData.front()->header.stamp.toSec() - sync_time > 0.1){     //  等待
            return false;
        }
        if(sync_time - UnsyncedData.front()->header.stamp.toSec() > 0.1){     //  太旧丢弃
            UnsyncedData.pop_front();
            continue;
        }
        break ;
    }
    if (UnsyncedData.size() < 1)
        return false;
    sensor_msgs::ImageConstPtr image_data = UnsyncedData.at(0);
    SyncedData.push_back(image_data);
    return true ;
}

bool OdomSyncData(std::deque<nav_msgs::Odometry::ConstPtr> &UnsyncedData,std::deque<nav_msgs::Odometry::ConstPtr> &SyncedData, double sync_time){
    while(UnsyncedData.size() >= 1){
        if(UnsyncedData.front()->header.stamp.toSec() - sync_time > 0.1){     //  等待
            return false;
        }
        if(sync_time - UnsyncedData.front()->header.stamp.toSec()  > 0.1){     //  太旧丢弃
            UnsyncedData.pop_front();
            continue;
        }
        break ;
    }
    if (UnsyncedData.size() < 1)
            return false;
    nav_msgs::Odometry::ConstPtr odom_data = UnsyncedData.at(0);
    SyncedData.push_back(odom_data);
    return true ;
}

bool SyncData()
{
	if(lidar_buff_.size() == 0) return false ;
  if(unsyced_image_.size() == 0) return false ;
  if(unsyced_odom_.size() == 0) return false ;
	// use timestamp of lidar measurement as reference:
    double cloud_time = lidar_buff_.front()->header.stamp.toSec();
    bool vaild_image = ImageSyncData(unsyced_image_, image_buff_, cloud_time);
    bool vaild_odom =  OdomSyncData(unsyced_odom_, odom_buff_, cloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!vaild_image || !vaild_odom) {
            lidar_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }
    if(lidar_buff_.size() == 0 || image_buff_.size() == 0 || odom_buff_.size() == 0) return false ;

    return true ;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "livox_color");
	ros::NodeHandle n;

	/*Load Parameter*/
	n.param<vector<double>>("mapping/extrinsicT", extrinT, vector<double>());	   	//   lidar2camera 外参
	n.param<vector<double>>("mapping/intrisicT", intrisicT, vector<double>());			  //   camera 内参
	n.param<vector<double>>("mapping/ditortion", ditortion, vector<double>());		//   	camera 畸变系数
	n.param<string>("common/lidar_topic", lidar_topic,"/cloud_registered");			//  sub 
	n.param<string>("common/lidar_color_topic", lidar_color_topic,"/livox/color_lidar");			// pub
	n.param<string>("common/odom_topic", odom_topic,"/Odometry");
    n.param<string>("common/camera_topic", camera_topic,"/camera/color/image_raw");
	n.param<string>("common/frame_id", frame_id,"camera_init");

	n.param<std::string>("savepcd/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");			//  保存地图路径

	CalibrationData();    // 加载标定参数

	image_transport::ImageTransport it(n);
	Twlidar = Eigen::Matrix4d::Identity() ;				//  初始化T为单位阵，如果没有SLAM的里程计输出，默认是固定在lidar系下 
	subOdom = n.subscribe(odom_topic, 1000, &OdomCallback);			// 订阅FASTLIO2 的里程计输出
  image_transport::Subscriber subImage = it.subscribe(camera_topic, 1000, &imageCallback); // 订阅图像
  subCloud = n.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1000, &pointCloudCallback);
  pubCloud = n.advertise<sensor_msgs::PointCloud2>(lidar_color_topic, 1);
  srvSaveMap  = n.advertiseService("/save_map" ,  &saveMapService);   	    // saveMap  发布地图保存服务
	ros::Rate loop_rate(500);

	while (ros::ok())
    {
        ros::spinOnce();
        if(SyncData()){
            ColorPointcloud();
        }
        loop_rate.sleep();
    }
	return 0 ;
}
