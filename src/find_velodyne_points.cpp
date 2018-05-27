#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "lidar_camera_calibration/Corners.h"
#include "lidar_camera_calibration/PreprocessUtils.h"
#include "lidar_camera_calibration/Find_RT.h"

#include "lidar_camera_calibration/marker_6dof.h"

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;


string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;


Mat projection_matrix;

pcl::PointCloud<myPointXYZRID> point_cloud;

Eigen::Quaterniond qlidarToCamera; 
Eigen::Matrix3d lidarToCamera;


void callback_noCam(const sensor_msgs::PointCloud2& msg_pc,
					const lidar_camera_calibration::marker_6dof& msg_rt
					)
{
	ROS_INFO_STREAM("Velodyne scan received at " << msg_pc.header.stamp.toSec());
	ROS_INFO_STREAM("marker_6dof received at " << msg_rt.header.stamp.toSec());

	// Loading Velodyne point cloud_sub
	fromROSMsg(msg_pc, point_cloud);

	 Eigen::Matrix4f T;
     for(int i=0;i<4;++i)
     	for(int j=0;j<4;++j)
     		T(i,j)=config.initialRot[3+4*i+j];
     	cout<<" new T\n"<<T<<endl;

    point_cloud = transform(point_cloud, config.tf[3], config.tf[4],config.tf[5], config.tf[0], config.tf[1],config.tf[2]);

	point_cloud = transform(point_cloud, 0, 0, 0, config.initialRot[0], config.initialRot[1], config.initialRot[2]);
	

	//Rotation matrix to transform lidar point cloud to camera's frame

	qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
		*Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

	lidarToCamera = qlidarToCamera.matrix();
    
	std:: cout << "\n\nInitial Rot\n" << lidarToCamera << "\n";
	point_cloud = intensityByRangeDiff(point_cloud, config);
	// x := x, y := -z, z := y


	//pcl::io::savePCDFileASCII ("/home/vishnu/PCDs/msg_point_cloud.pcd", pc);  


	cv::Mat temp_mat(config.s, CV_8UC3);
	pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

	std::vector<float> marker_info;

	for(std::vector<float>::const_iterator it = msg_rt.dof.data.begin(); it != msg_rt.dof.data.end(); ++it)
	{
		marker_info.push_back(*it);
		std::cout << *it << " ";
	}
	std::cout << "\n";

	getCorners(temp_mat, retval, config.P, config.num_of_markers, config.MAX_ITERS);
	find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
	//ros::shutdown();
}

void callback_test(const sensor_msgs::PointCloud2& msg_pc, sensor_msgs::Image & image)
{
	// Loading Velodyne point cloud_sub
	fromROSMsg(msg_pc, point_cloud);

     Eigen::Matrix4f T;
     for(int i=0;i<4;++i)
     	for(int j=0;j<4;++j)
     		T(i,j)=config.initialRot[3+4*i+j];
     	cout<<" new T"<<T<<endl;
    //Lidar2Vehilce
    point_cloud = transform(point_cloud, config.tf[3], config.tf[4],config.tf[5], config.tf[0], config.tf[1],config.tf[2]);
    //Vehicle2Camera
	point_cloud = transform(point_cloud, 0, 0, 0, config.initialRot[0], config.initialRot[1], config.initialRot[2]);
    //Calibration
    point_cloud = transform(point_cloud, T);

	//Rotation matrix to transform lidar point cloud to camera's frame
	point_cloud = intensityByRangeDiff(point_cloud, config);
	// x := x, y := -z, z := y

	cv::Mat temp_mat(config.s, CV_8UC3);
	pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));
	Test(image,temp_mat, retval, config.P, config.num_of_markers, config.MAX_ITERS);
	ros::shutdown();
}

bool laserFlag=false;
bool markerFlag=false;
bool imageFlag=false;
sensor_msgs::PointCloud2 velo16;
lidar_camera_calibration::marker_6dof marker;
sensor_msgs::Image lidar2image;

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//if(!laserFlag)
	{
      laserFlag=true;
      velo16=*msg;
	}
  
 }

 void markerCallback(const lidar_camera_calibration::marker_6dof::ConstPtr& msg)
{
	//if(!markerFlag)
	{
      markerFlag=true;
      marker=*msg;
	}
  
 }

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	//if(!laserFlag)
	{
      imageFlag=true;
      lidar2image=*msg;
     }
 }

int main(int argc, char** argv)
{
	readConfig();
	ros::init(argc, argv, "find_transform");

	ros::NodeHandle n;

	
		ROS_INFO_STREAM("Reading CameraInfo from configuration file");
  		n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);

         ros::Subscriber sublaser = n.subscribe("velodyne_points", 1000, laserCallback);
         ros::Subscriber submarker = n.subscribe("lidar_camera_calibration_rt", 1000, markerCallback);
         ros::Subscriber subimage = n.subscribe("camera/image_raw", 1000, imageCallback);
         
       while(true)
       {
       	  if(laserFlag&&markerFlag&&imageFlag)
        	{
        		if(config.test)
               callback_test(velo16,lidar2image);
              else
               callback_noCam(velo16,marker);
            }
            ros::spinOnce();
       }
		ros::spin();

	return EXIT_SUCCESS;
}