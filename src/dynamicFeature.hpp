#ifndef __DYNAMIC__
#define __DYNAMIC__

#include "utility.h"
#include "dynamic/information.h"
#include "DBSCAN/dbscan.h"
#include "DBSCAN/dbscan.cpp"

// point with index
typedef struct{
	PointType midPoint;
	int startIdx;
	int endIdx;
}IntervalPoint;

// frameData
typedef struct{
	std_msgs::Header header;
	pcl::PointCloud<PointType>::Ptr cloudData;
}FrameData;

class DynamicFeature : public ParamServer{
	public:
		DynamicFeature();
		~DynamicFeature();
		void informationHandler(const dynamic::information::ConstPtr& infoMsg);

		void generatedImuHandler(const nav_msgs::Odometry::ConstPtr& imuMsg);

		void generatedOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);

	private:
		ros::Subscriber subInformation;
		ros::Subscriber subGeneratedOdometry;
		ros::Subscriber subGeneratedImu;

		ros::Publisher pubStaticPoints;
		ros::Publisher pubDynamicPoints;
		ros::Publisher pubTempPointCloud;
		ros::Publisher pubPostPointCloud;

		std::deque<nav_msgs::Odometry> generatedImuDataQueue;
		std::deque<nav_msgs::Odometry> generatedOdomDataQueue;
		std::deque<FrameData> frameDataQueue;

		pcl::PointCloud<VelodynePointType>::Ptr velodyneCloud;
		pcl::PointCloud<OusterPointType>::Ptr ousterCloud;
		pcl::PointCloud<LivoxPointType>::Ptr livoxCloud;

		pcl::PointCloud<PointType>::Ptr extractedCloud;
		pcl::PointCloud<PointType>::Ptr predictedDynamicPoints;
		pcl::PointCloud<PointType>::Ptr contourCloud;

		std_msgs::Header cloudHeader;
		dynamic::information info;

		Dbscan *ds;

		std::mutex mtx;
		std::mutex imuMtx;
		std::mutex odomMtx;

		std::vector<std::pair<int, int>> idxContainer;
		std::multimap<int, IntervalPoint> tempClusterContainer;
		std::multimap<int, IntervalPoint> clusterContainer;

		std::vector<int> clusterIdx;
		bool *dynamicPointArray;

		int clusterNum;
		bool isFrameStackUp;
		bool isOdomGenerated;

		void findPoints();
		void resetParam();
		void extractContour();
		void clustering2D();
		void divideCluster();
		void revertCluster();
		void saveFrame();

		void cloudTransform(double pose[], pcl::PointCloud<PointType>::Ptr src, pcl::PointCloud<PointType>::Ptr dst);
		void syncLidarOdom();
		void extendedTargetTracking();
		void findUnmatchedPoints();
		void extractStaticPoints();
		void publishPointCloud();
};
#endif
