#ifndef __PRE_PROCESSING__
#define __PRE_PROCESSING__

#include "utility.h"
#include "dynamic/information.h"

class PreProcessing : public ParamServer{
	public:
		PreProcessing();
		~PreProcessing();
		void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg);

	private:
		ros::Subscriber subPointCloud;
		ros::Publisher pubPointCloud;
		ros::Publisher pubInformation;

		std::deque<sensor_msgs::PointCloud2> cloudDataQueue;
		sensor_msgs::PointCloud2 cloudMsg;

		std_msgs::Header cloudHeader;
		dynamic::information info;
		pcl::PointCloud<PointType>::Ptr extractedCloud;

		pcl::PointCloud<VelodynePointType>::Ptr velodyneCloud;
		pcl::PointCloud<OusterPointType>::Ptr ousterCloud;
		pcl::PointCloud<LivoxPointType>::Ptr livoxCloud;

		ros::Publisher pubPrePointCloud;

		void resetParam();
		bool convertCloudFormat(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg);
		bool checkValidCloud();
		void saveCloudInformation();
		void publishCloudInfo();
};
#endif
