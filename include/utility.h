#define PCL_NO_PRECOMPILE

#ifndef __UTILTIY__
#define __UTILTIY__

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <functional>
#include <chrono>

//velodyne
struct VelodynePointType{
	PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointType,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

//ouster
struct OusterPointType{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
	//uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointType,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) /*(uint16_t, noise, noise)*/ (uint32_t, range, range)
)

//livox
struct LivoxPointType{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t reflectivity;
    uint8_t tag;
    uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointType,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, reflectivity, reflectivity) (uint8_t, tag, tag)
	(uint8_t, line, line)
)

using namespace std::chrono;
using PointType = pcl::PointXYZI;

enum class Sensor {VELODYNE, OUSTER, LIVOX};

class ParamServer{
	public:
		ParamServer();
		~ParamServer();

		ros::NodeHandle nh;

		std::string cloudTopic;
		std::string rawOdomTopic;
		std::string generatedOdomTopic;
		std::string imuTopic;

		int channel;
		int resolution;

		float bias_p;
		float bias_q;
		float bias_r;

		float imuFreq;
		float odomFreq;
		float lidarFreq;

		float minRange;
		float maxRange;
		
		int minPoints;
		float epslion;
		float distThreshold;
		float distDiffThreshold;
		float distClusterThreshold;

		std::string sensorName;
		std::string refFrame;
		Sensor sensor;
};

ParamServer::ParamServer()
{
	//sensor
    nh.param<std::string>("cloudTopic", cloudTopic, "os_cloud_node/points");
    nh.param<std::string>("rawOdomTopic", rawOdomTopic, "jackal_velocity_controller/odom");
    nh.param<std::string>("generatedOdomTopic", generatedOdomTopic, "lio_sam/mapping/odometry_incremental");
    nh.param<std::string>("imuTopic", imuTopic, "gx5/imu/data");
    nh.param<std::string>("refFrame", refFrame, "os_sensor");

    nh.param<int>("channel", channel, 64);
    nh.param<int>("resolution", resolution, 1024);
	
	nh.param<std::string>("sensorName", sensorName, "ouster");
	if(sensorName == "velodyne")
		sensor = Sensor::VELODYNE;
	else if(sensorName == "ouster")
		sensor = Sensor::OUSTER;
	else if(sensorName == "livox")
		sensor = Sensor::LIVOX;

	// imu 100hz
    nh.param<float>("bias_p", bias_p, 0);
    nh.param<float>("bias_q", bias_q, 0);
    nh.param<float>("bias_r", bias_r, 0);
    nh.param<float>("imuFreq", imuFreq, 0.01);
	
	// odom 50hz
    nh.param<float>("odomFreq", odomFreq, 0.02);

	// lidar 10hz
    nh.param<float>("lidarFreq", lidarFreq, 0.1);

	// preprocessing
    nh.param<float>("minRange", minRange, 1.0);
    nh.param<float>("maxRange", maxRange, 300.0);

	// dbscan
	nh.param<int>("/minPoints", minPoints, 5);
	nh.param<float>("/epslion", epslion, 0.1);

	// dynamicFeature
	nh.param<float>("/distDiffThreshold", distDiffThreshold, 0.3);
	nh.param<float>("/distThreshold", distThreshold, 15.0);
	nh.param<float>("/distClusterThreshold", distClusterThreshold, 0.2);
}

ParamServer::~ParamServer(){}

template<typename PC1, typename PC2>
void convertCloud(PC1 srcCloud, PC2 dstCloud)
{
	dstCloud->is_dense = srcCloud->is_dense;
	for (int i = 0; i < srcCloud->size(); i++){
		auto &srcPoint = srcCloud->points[i];
		auto &dstPoint = dstCloud->points[i];
		dstPoint.x = srcPoint.x;
		dstPoint.y = srcPoint.y;
		dstPoint.z = srcPoint.z;
		dstPoint.intensity = srcPoint.intensity;
	}
}

template<typename PC>
void publishCloud(const ros::Publisher& publisher, const PC& pointCloud, ros::Time stamp, std::string frame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*pointCloud, tempCloud);
    tempCloud.header.stamp = stamp;
    tempCloud.header.frame_id = frame;
    publisher.publish(tempCloud);
}

float timeCheck(const std::function<void(void)>& fn)
{
	system_clock::time_point start_time = system_clock::now();
	fn();
	system_clock::time_point end_time = system_clock::now();
	nanoseconds nano = (end_time-start_time);
 
	return double(nano.count())/1e9;
}

inline float distance2D(PointType& p)
{
    return sqrt(p.x*p.x + p.y*p.y);
}

inline float distance3D(PointType& p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline float pointDistance2D(PointType& p1, PointType& p2)
{
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

inline float pointDistance3D(PointType& p1, PointType& p2)
{
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
}

inline float pointAngle2D(PointType& p1, PointType& p2)
{
	float a1 = std::atan2(p1.y, p1.x);
	float a2 = std::atan2(p2.y, p2.x);

	float angle = std::abs(a1 - a2) * 180 / M_PI;

	return angle;
}
#endif
