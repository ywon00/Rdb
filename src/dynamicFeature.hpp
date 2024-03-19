#ifndef __DYNAMIC__
#define __DYNAMIC__

#include "utility.h"
#include "dynamic/information.h"
#include "DBSCAN/dbscan.h"
#include "DBSCAN/dbscan.cpp"

// point with eigen
struct EigenPointType{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
	float normalX;
	float normalY;
	float normalZ;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(EigenPointType,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, normalX, normalX) (float, normalY, normalY) (float, normalZ, normalZ)
)

// point with index
typedef struct{
	PointType midPoint;
	int startIdx;
	int endIdx;
}IntervalPoint;

// global pose / wheel, imu
typedef struct{
	nav_msgs::Odometry odomData;
	sensor_msgs::Imu imuData;
	double DOF[6];
}PoseData;

// frameData
typedef struct{
	std_msgs::Header header;
	pcl::PointCloud<EigenPointType> pointData;
	double DOF[6];
}FrameData;

class DynamicFeature : public ParamServer{
	public:
		DynamicFeature();
		~DynamicFeature();
		void informationHandler(const dynamic::information::ConstPtr& infoMsg);

		void rawImuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg);
		void generatedImuHandler(const nav_msgs::Odometry::ConstPtr& imuMsg);

		void generatedOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
		void rawOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);

	private:
		ros::Subscriber subPointCloud;
		ros::Subscriber subRawOdometry;
		ros::Subscriber subGeneratedOdometry;
		ros::Subscriber subRawImu;
		ros::Subscriber subGeneratedImu;

		ros::Publisher pubPreDynamicPointCloud;
		ros::Publisher pubCmpDynamicPointCloud;
		ros::Publisher pubClusterPoints;
		ros::Publisher pubDescriptorPoints;
		ros::Publisher pubStaticPoints;
		ros::Publisher pubDynamicPoints;
		ros::Publisher pubTempPointCloud;
		ros::Publisher pubPostPointCloud;

		ros::Publisher arrowPub;

		std::deque<sensor_msgs::Imu> imuDataQueue;
		std::deque<nav_msgs::Odometry> generatedImuDataQueue;
		std::deque<PoseData> poseDataQueue;
		std::deque<FrameData> frameDataQueue;
		std::deque<pcl::PointCloud<PointType>::Ptr> cloudQueue;

		pcl::PointCloud<VelodynePointType>::Ptr velodyneCloud;
		pcl::PointCloud<OusterPointType>::Ptr ousterCloud;
		pcl::PointCloud<LivoxPointType>::Ptr livoxCloud;

		pcl::PointCloud<PointType>::Ptr extractedCloud;
		pcl::PointCloud<PointType>::Ptr predictedDynamicPoints;
		pcl::PointCloud<PointType>::Ptr clusterPoints;
		pcl::PointCloud<PointType>::Ptr wholePoints;

		pcl::PointCloud<EigenPointType>::Ptr preDynamicPoints;
		pcl::PointCloud<EigenPointType>::Ptr cmpDynamicPoints;
		pcl::PointCloud<EigenPointType>::Ptr descriptorPoints;
		pcl::KdTreeFLANN<EigenPointType>::Ptr kdtreeDescriptor;

		std_msgs::Header cloudHeader;
		dynamic::information info;
		double currGeneratedOdomTime;

		Dbscan *ds;

		std::mutex mtx;
		std::mutex imuMtx;
		std::mutex odomMtx;

		std::vector<std::pair<int, int>> idxContainer;
		std::multimap<int, IntervalPoint> tempClusterContainer;
		std::multimap<int, IntervalPoint> clusterContainer;

		std::vector<int> clusterIdx;
		bool *dynamicPointArray;

		// x y z roll pitch yaw
		double global6DOF[6];
		double curr6DOF[6];
		int clusterNum;
		bool isFrameStackUp;
		bool isOdomGenerated;

		void findPoints();
		void resetParam();
		void estimateLidarPose();
		void extractContour();
		void clustering2D();
		void divideCluster();
		void genClusterPlane();
		void saveFrame();

		void estimateMotion(PoseData& prePoseData, double dt, double pose[] = 0, bool inv = false);
		void cloudTransform(double pose[], pcl::PointCloud<EigenPointType>::Ptr src, pcl::PointCloud<EigenPointType>::Ptr dst);
		void imuEncoderPropagation();
		void findUnmatchedPoints();
		void extractStaticPoints();
		void createBoundingBox();
		void publishPointCloud();
};
#endif
