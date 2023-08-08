#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std ;

#define Hmax 720
#define Wmax 1280
#define H Hmax
#define W Wmax

Eigen::Isometry3d  Twlidar  ;			//  FASTLIO2 订阅当前里程计
vector<double>       extrinT(16, 0.0);				//  lidar2camera
vector<double>       intrisicT(9, 0.0);				//   camera  intrinT
vector<double>       ditortion(5, 0.0);				//  畸变系数

string  lidar_topic, lidar_color_topic, camera_topic, odom_topic, frame_id ;

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

class livox_lidar_color
{
public:
	ros::NodeHandle n;
	sensor_msgs::PointCloud2 msg;																									   //接收到的点云消息
	sensor_msgs::PointCloud2 fusion_msg;																							   //等待发送的点云消息
	ros::Subscriber subCloud = n.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &livox_lidar_color::pointCloudCallback, this); //接收点云数据，进入回调函数pointCloudCallback
	ros::Publisher pubCloud = n.advertise<sensor_msgs::PointCloud2>(lidar_color_topic, 1);										   //建立了一个发布器，方便之后发布加入颜色之后的点云

private:
	//点云回调函数
	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
	{
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
		// std::cout<<  fusion_pcl_ptr->points.size() << std::endl;
		pcl::toROSMsg(*fusion_pcl_ptr, fusion_msg);			   //将点云转化为消息才能发布
		fusion_msg.header.frame_id = frame_id ;			   //   Fastlio2 中 frame-id 为 camera_init
		fusion_msg.header.stamp = laserCloudMsg->header.stamp; // 时间戳和/livox/lidar 一致
		pubCloud.publish(fusion_msg);						   //发布调整之后的点云数据
	}
};

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; //image_raw就是我们得到的图像了
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
		ROS_ERROR("Could not conveextrinsicMat_RT from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{ 
	Eigen::Vector3d t_w_lidar(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);	 //    当前载体在世界系下的位置
	Eigen::Quaterniond q_w_lidar(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);   //    当前载体在世界系下的姿态
	q_w_lidar.normalize();
	Eigen::Isometry3d T_w_lidar(q_w_lidar);
	T_w_lidar.pretranslate(t_w_lidar); 
	Twlidar = T_w_lidar ;
    // ROS_INFO("Odom: %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, yaw);
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

	livox_lidar_color llc;

	CalibrationData();

	image_transport::ImageTransport it(n);
	Twlidar = Eigen::Matrix4d::Identity() ;				//  初始化T为单位阵，如果没有SLAM的里程计输出，默认是固定在lidar系下 
	ros::Subscriber subOdom = n.subscribe(odom_topic, 1000, &OdomCallback);			// 订阅FASTLIO2 的里程计输出
	image_transport::Subscriber sub = it.subscribe(camera_topic, 1000, &imageCallback);
	ros::Rate loop_rate(200);

	while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
	// ros::spin();
	return 0 ;
}
