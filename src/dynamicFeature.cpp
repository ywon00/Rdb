#include "dynamicFeature.hpp" 

DynamicFeature::DynamicFeature()
{
	subPointCloud = nh.subscribe<dynamic::information>("information", 1, &DynamicFeature::informationHandler, this, ros::TransportHints().tcpNoDelay());

	subRawOdometry = nh.subscribe<nav_msgs::Odometry>(rawOdomTopic, 10, &DynamicFeature::rawOdometryHandler, this, ros::TransportHints().tcpNoDelay());
	subGeneratedOdometry = nh.subscribe<nav_msgs::Odometry>(generatedOdomTopic, 10, &DynamicFeature::generatedOdometryHandler, this, ros::TransportHints().tcpNoDelay());

	subRawImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 100, &DynamicFeature::rawImuHandler, this, ros::TransportHints().tcpNoDelay());
	subGeneratedImu = nh.subscribe<nav_msgs::Odometry>("/odometry/imu_incremental", 100, &DynamicFeature::generatedImuHandler, this, ros::TransportHints().tcpNoDelay());

	pubPreDynamicPointCloud = nh.advertise<sensor_msgs::PointCloud2>("pre_dynamic_points", 1);
	pubCmpDynamicPointCloud = nh.advertise<sensor_msgs::PointCloud2>("cmp_dynamic_points", 1);
	pubClusterPoints = nh.advertise<sensor_msgs::PointCloud2>("cluster_points", 1);
	pubDescriptorPoints = nh.advertise<sensor_msgs::PointCloud2>("descriptor_points", 1);
	pubStaticPoints = nh.advertise<sensor_msgs::PointCloud2>("static_points", 1);
	pubDynamicPoints = nh.advertise<sensor_msgs::PointCloud2>("dynamic_points", 1);
	arrowPub = nh.advertise<visualization_msgs::MarkerArray>("planar", 10);
        
	pubTempPointCloud = nh.advertise<sensor_msgs::PointCloud2>("whole_cloud", 1);
	pubPostPointCloud = nh.advertise<sensor_msgs::PointCloud2>("post_cloud", 1);

	velodyneCloud.reset(new pcl::PointCloud<VelodynePointType>());
	ousterCloud.reset(new pcl::PointCloud<OusterPointType>());
	livoxCloud.reset(new pcl::PointCloud<LivoxPointType>());

	ds = new Dbscan(minPoints, epslion);
	extractedCloud.reset(new pcl::PointCloud<PointType>());
	predictedDynamicPoints.reset(new pcl::PointCloud<PointType>());
	clusterPoints.reset(new pcl::PointCloud<PointType>());
	wholePoints.reset(new pcl::PointCloud<PointType>());

	preDynamicPoints.reset(new pcl::PointCloud<EigenPointType>());
	cmpDynamicPoints.reset(new pcl::PointCloud<EigenPointType>());
	descriptorPoints.reset(new pcl::PointCloud<EigenPointType>());

	kdtreeDescriptor.reset(new pcl::KdTreeFLANN<EigenPointType>());
	dynamicPointArray = new bool[channel*resolution];

	isOdomGenerated = false;
	isFrameStackUp = false;
	currGeneratedOdomTime = -1;
	std::fill(global6DOF, global6DOF + 6, 0);
	std::fill(curr6DOF, curr6DOF + 6, 0);
}

DynamicFeature::~DynamicFeature(){}

void DynamicFeature::resetParam()
{
	predictedDynamicPoints->clear();
	clusterPoints->clear();
	descriptorPoints->clear();
	wholePoints->clear();

	idxContainer.clear();
	tempClusterContainer.clear();
	clusterContainer.clear();
	clusterIdx.clear();

	clusterNum = 0;
}

void DynamicFeature::informationHandler(const dynamic::information::ConstPtr& infoMsg)
{
	static int pre = 1;
	static float max_process_time = 0;
	static float pre_process_time = 0;

	info = *infoMsg;
	cloudHeader = infoMsg->header;
	pcl::fromROSMsg(infoMsg->extract_cloud, *extractedCloud);

	// function
	float process_time = timeCheck(std::bind(&DynamicFeature::findPoints, this));

	max_process_time = std::max(max_process_time, process_time);
	printf("max process time: %f\n", max_process_time);

	float a = float(pre - 1) / pre;
	pre++;
	float avg_time = a * pre_process_time + (1 - a) * process_time;
	pre_process_time = avg_time;
	printf("avg process time: %f\n\n", avg_time);
}

void DynamicFeature::rawImuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
	if(isOdomGenerated)
		return;

	std::lock_guard<std::mutex> lock(imuMtx);
	imuDataQueue.push_back(*imuMsg);
}

// lio-sam odometry
void DynamicFeature::generatedImuHandler(const nav_msgs::Odometry::ConstPtr& odomImuMsg)
{
	std::lock_guard<std::mutex> lock(imuMtx);
	generatedImuDataQueue.push_back(*odomImuMsg);
}

// lio-sam odometry
void DynamicFeature::generatedOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
	isOdomGenerated = true;

	double roll, pitch, yaw;
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
	tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

	global6DOF[3] = roll;
	global6DOF[4] = pitch;
	global6DOF[5] = yaw;
	
	global6DOF[0] = odomMsg->pose.pose.position.x;
	global6DOF[1] = odomMsg->pose.pose.position.y;
	global6DOF[2] = odomMsg->pose.pose.position.z;

	currGeneratedOdomTime = odomMsg->header.stamp.toSec();

	/*
	std::cout << "lio-sam" << "\n";

	for(int i = 0; i < 6; i++)
		std::cout << global6DOF[i] << " ";
	std::cout << "\n";
	*/
}

void DynamicFeature::rawOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
	static bool isFirstOdom = true;
	static nav_msgs::Odometry preOdomData;
	
	if(!isOdomGenerated){
		if(isFirstOdom){
			preOdomData = *odomMsg;
			isFirstOdom = false;

			return;
		}

		if(imuDataQueue.empty())
			return;

		while(!imuDataQueue.empty()){
			if(imuDataQueue.front().header.stamp.toSec() <= preOdomData.header.stamp.toSec())
				imuDataQueue.pop_front();
			else
				break;
		}

		double currOdomTime = odomMsg->header.stamp.toSec();

			double dt = currOdomTime - preOdomData.header.stamp.toSec();

			PoseData prePoseData;
			std::copy(global6DOF, global6DOF + 6, prePoseData.DOF);
			prePoseData.odomData = preOdomData;
			prePoseData.imuData = imuDataQueue.front();

			// global
			estimateMotion(prePoseData, dt);
	}

	PoseData currPoseData;
	std::copy(global6DOF, global6DOF + 6, currPoseData.DOF);
	currPoseData.odomData = *odomMsg;
	currPoseData.imuData = imuDataQueue.back();

	poseDataQueue.push_back(currPoseData);

	preOdomData = *odomMsg;

	/*
	std::cout << "raw" << "\n";

	for(int i = 0; i < 6; i++)
		std::cout << global6DOF[i] << " ";
	std::cout << "\n";
	*/
}

inline void DynamicFeature::estimateMotion(PoseData& prePoseData, double dt, double pose[]/* = 0*/, bool inv/* = false*/)
{
	// deadreckoning
	// X
	double roll, pitch, yaw;
	roll = prePoseData.DOF[3];
	pitch = prePoseData.DOF[4];
	yaw = prePoseData.DOF[5];

	//U
	double lin_x, lin_y, lin_z, ang_x, ang_y, ang_z;
	lin_x = prePoseData.odomData.twist.twist.linear.x;
	lin_y = prePoseData.odomData.twist.twist.linear.y;
	lin_z = prePoseData.odomData.twist.twist.linear.z;
	ang_x = prePoseData.imuData.angular_velocity.x;
	ang_y = prePoseData.imuData.angular_velocity.y;
	ang_z = prePoseData.imuData.angular_velocity.z;

	// rotation matrix
	Eigen::Matrix3d rotMat;
	rotMat << cos(yaw)*cos(pitch), sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
			  cos(pitch)*sin(yaw), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
			  -sin(pitch), sin(roll)*cos(pitch), cos(pitch)*cos(roll);

	// transpose rotation matrix
	if(inv)
		rotMat.transpose();

	double x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot;
	x_dot = rotMat(0,0)*lin_x + rotMat(0,1)*lin_y + rotMat(0,2)*lin_z;
	y_dot = rotMat(1,0)*lin_x + rotMat(1,1)*lin_y + rotMat(1,2)*lin_z;
	z_dot = rotMat(2,0)*lin_x + rotMat(2,1)*lin_y + rotMat(2,2)*lin_z;
	phi_dot = (ang_x-bias_p) + (tan(pitch)*sin(roll)*(ang_y-bias_q) + tan(pitch)*cos(roll)*(ang_z-bias_r));
	theta_dot = cos(roll)*(ang_y-bias_q) - sin(roll)*(ang_z-bias_r);
	psi_dot = sin(roll)*(ang_y-bias_q)/cos(pitch) + cos(roll)*(ang_z-bias_r)/cos(pitch);

	// global pose
	if(!pose){
		global6DOF[0] += x_dot * dt;
		global6DOF[1] += y_dot * dt;
		global6DOF[2] += z_dot * dt;

		global6DOF[3] += phi_dot * dt;
		global6DOF[4] += theta_dot * dt;
		global6DOF[5] += psi_dot * dt;
	}
	// other pose
	else{
		pose[0] += x_dot * dt;
		pose[1] += y_dot * dt;
		pose[2] += z_dot * dt;

		pose[3] += phi_dot * dt;
		pose[4] += theta_dot * dt;
		pose[5] += psi_dot * dt;
	}
}

void DynamicFeature::findPoints()
{
	std::lock_guard<std::mutex> lock(mtx);

	resetParam();

	estimateLidarPose();

	extractContour();

	clustering2D();

	divideCluster();

	genClusterPlane();

	saveFrame();

	if(isFrameStackUp){
		imuEncoderPropagation();

		findUnmatchedPoints();

		extractStaticPoints();

		publishPointCloud();
	}
}

void DynamicFeature::estimateLidarPose()
{
	if(isOdomGenerated){
		cout.precision(10);
		cout << cloudHeader.stamp <<"\n";
		cout << currGeneratedOdomTime <<"\n";
	   if(cloudHeader.stamp.toSec() - currGeneratedOdomTime < 0.31){
			std::copy(global6DOF, global6DOF + 6, curr6DOF);

			std::cout << "same" <<"\n";
			for(int i = 0; i < 6; i++)
				std::cout << curr6DOF[i] << " ";
			std::cout << "\n";
	   }
	   else{
		   nav_msgs::Odometry nearestImuData;
		   while(!generatedImuDataQueue.empty()){
			   if(generatedImuDataQueue.front().header.stamp.toSec() <= cloudHeader.stamp.toSec()){
				   nearestImuData = std::move(generatedImuDataQueue.front());
				   generatedImuDataQueue.pop_front();
		}
			   else
				   break;
		   }

		   // modify necessary
		   PoseData nearestPrePoseData;

		   nearestPrePoseData.odomData.twist.twist.linear.x = nearestImuData.twist.twist.linear.x;
		   nearestPrePoseData.odomData.twist.twist.linear.y = nearestImuData.twist.twist.linear.y;
		   nearestPrePoseData.odomData.twist.twist.linear.z = nearestImuData.twist.twist.linear.z;

		   nearestPrePoseData.imuData.angular_velocity.x = nearestImuData.twist.twist.angular.x;
		   nearestPrePoseData.imuData.angular_velocity.y = nearestImuData.twist.twist.angular.y;
		   nearestPrePoseData.imuData.angular_velocity.z = nearestImuData.twist.twist.angular.z;

		   double dt = cloudHeader.stamp.toSec() - nearestImuData.header.stamp.toSec();
		   estimateMotion(nearestPrePoseData, dt, curr6DOF);

		   std::cout << "lidar" << "\n";

		   for(int i = 0; i < 6; i++)
			   std::cout << curr6DOF[i] << " ";
		   std::cout << "\n";
	   }

	   return;
	}
				
	// dead reckoning
	while(!poseDataQueue.empty()){
		if(poseDataQueue.front().odomData.header.stamp.toSec() <= cloudHeader.stamp.toSec() - 0.1){
			poseDataQueue.pop_front();
		}
		else
			break;
	}

		PoseData prePoseData = poseDataQueue.front();
		std::copy(curr6DOF, curr6DOF + 6, prePoseData.DOF);

		double dt = cloudHeader.stamp.toSec() - prePoseData.odomData.header.stamp.toSec();

		estimateMotion(prePoseData, dt, curr6DOF);

	std::cout << "estimate" << "\n";

	for(int i = 0; i < 6; i++)
		std::cout << curr6DOF[i] << " ";
	std::cout << "\n";
}

void DynamicFeature::extractContour()
{
	bool isStart = false, isEnd = false, isCorrectSeq;
	int startIdx, endIdx;

	for (int i = 1; i < extractedCloud->points.size()-1; i++){
		float dist1 = info.point_range_2D[i - 1];
		float dist2 = info.point_range_2D[i];
		float dist3 = info.point_range_2D[i + 1];

		bool isFarLeftFront = (dist2 - dist1) > distDiffThreshold;
		bool isFarLeftBack = (dist1 - dist2) > distDiffThreshold;
		bool isFarRightFront = (dist2 - dist3) > distDiffThreshold;
		bool isFarRightBack = (dist3 - dist2) > distDiffThreshold;

		if(dist2 < distThreshold){
			if(isFarLeftFront){
				startIdx = i;
				isStart = true;
				isCorrectSeq = false;
			}
			else if(isFarLeftBack){
				startIdx = i;
				isStart = true;
				isCorrectSeq = false;
			}
			else if(isFarRightFront){
				endIdx = i;
				isEnd = true;
				isCorrectSeq = true;
			}
			else if(isFarRightBack){
				endIdx = i;
				isEnd = true;
				isCorrectSeq = true;
			}
		}

		if(isStart && isEnd && isCorrectSeq){
			isStart = false;
			isEnd = false;

			float cDistance = 0;
			for(int j = startIdx; j < endIdx; j++){
				cDistance += pointDistance2D(extractedCloud->points[j], 
											 extractedCloud->points[j+1]);
			}
			float uDistance = pointDistance2D(extractedCloud->points[startIdx], extractedCloud->points[endIdx]);	
			
			if(cDistance / uDistance > 3.0)
				continue;

			if(pointAngle2D(extractedCloud->points[startIdx], extractedCloud->points[endIdx]) > 160)
				continue;

			if(endIdx - startIdx > 1){
				PointType point;
				point.x = (extractedCloud->points[startIdx].x + extractedCloud->points[endIdx].x)/2;
				point.y = (extractedCloud->points[startIdx].y + extractedCloud->points[endIdx].y)/2;
				point.z = (extractedCloud->points[startIdx].z + extractedCloud->points[endIdx].z)/2;

				predictedDynamicPoints->emplace_back(point);
				idxContainer.emplace_back(std::make_pair(startIdx, endIdx));

				wholePoints->emplace_back(extractedCloud->points[startIdx]);
				wholePoints->emplace_back(extractedCloud->points[endIdx]);
			}
		}
	}

	info.point_range_2D.clear();
}

void DynamicFeature::clustering2D()
{
	int pointSize = predictedDynamicPoints->points.size();

	vector<Point> points;
	Point *p = new Point[pointSize];
	for(int i = 0; i < pointSize; i++){
		p[i].clusterID = UNCLASSIFIED;
		p[i].x = predictedDynamicPoints->points[i].x;
		p[i].y = predictedDynamicPoints->points[i].y;
		points.push_back(p[i]);
	}
	delete p;

	ds->dbscan(points);

	for(int i = 0; i < pointSize; i++){
		int clusterID = points[i].clusterID;
		clusterNum = std::max(clusterNum, clusterID);

		if(clusterID > 0){
			predictedDynamicPoints->points[i].intensity = clusterID;

			wholePoints->points[2*i].intensity = clusterID;
			wholePoints->points[2*i+1].intensity = clusterID;

			IntervalPoint itv;
			itv.midPoint = predictedDynamicPoints->points[i];
			itv.startIdx = idxContainer[i].first;
			itv.endIdx = idxContainer[i].second;

			tempClusterContainer.emplace(clusterID, itv);
		}
		else{
			predictedDynamicPoints->points[i].intensity = -10;
			wholePoints->points[2*i].intensity = -10;
			wholePoints->points[2*i+1].intensity = -10;

		}
	}
}

void DynamicFeature::divideCluster()
{
	int newClusterID = 0;
	for(int i = 1; i <= clusterNum; i++){
		auto rangeIter = tempClusterContainer.equal_range(i);

		std::vector<IntervalPoint> tempClusterCloud;
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			if(iter != rangeIter.first &&
			std::prev(iter)->second.midPoint.z - iter->second.midPoint.z > 0.2){
				newClusterID += 1;
				for (auto& itv : tempClusterCloud) {
					itv.midPoint.intensity = newClusterID;
					clusterContainer.emplace(newClusterID, itv);
					clusterPoints->emplace_back(itv.midPoint);
				}
				tempClusterCloud.clear();
			}
			tempClusterCloud.emplace_back(iter->second);
		}

		newClusterID += 1;
		for (auto& itv : tempClusterCloud) {
			itv.midPoint.intensity = newClusterID;
			clusterContainer.emplace(newClusterID, itv);
			clusterPoints->emplace_back(itv.midPoint);
		}
	}

	clusterNum = newClusterID;
}

void DynamicFeature::genClusterPlane(){
	visualization_msgs::MarkerArray arrowArray; 

	for(int i = 1; i <= clusterNum; i++){
		int count = clusterContainer.count(i);

		if(count <= 5)
			continue;

		auto rangeIter = clusterContainer.equal_range(i);

		// xyz average, centroid
		float x = 0, y = 0, z = 0;
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			x += iter->second.midPoint.x;
			y += iter->second.midPoint.y;
			z += iter->second.midPoint.z;
		}
		x /= count;
		y /= count;
		z /= count;

		// least square, plane
		float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			int startIdx = iter->second.startIdx;
			int endIdx = iter->second.endIdx;

			float dx[2];
			float dy[2];
			float dz[2];

			dx[0] = extractedCloud->points[startIdx].x - x;
			dy[0] = extractedCloud->points[startIdx].y - y;
			dz[0] = extractedCloud->points[startIdx].z - z;
			dx[1] = extractedCloud->points[endIdx].x - x;
			dy[1] = extractedCloud->points[endIdx].y - y;
			dz[1] = extractedCloud->points[endIdx].z - z;

			for(int j = 0; j < 2; j++){
				xx += dx[j] * dx[j];
				xy += dx[j] * dy[j];
				xz += dx[j] * dz[j];
				yy += dy[j] * dy[j];
				yz += dy[j] * dz[j];
				zz += dz[j] * dz[j];
			}
		}

		// covariance
		Eigen::Matrix3f covMat;
		covMat << xx, xy, xz,
				  xy, yy, yz,
				  xz, yz, zz;

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covMat);
		Eigen::Vector3f normal = es.eigenvectors().col(0).normalized();

		/*
		std::cout << "\n" << i << "\n";
		std::cout << "x y z: " << x << " " << y << " " << z << "\n";
		std::cout << "Eigenvalues: \n" << es.eigenvalues() << "\n";
		std::cout << "Eigenvectors: \n" << es.eigenvectors() << "\n";
		std::cout << "normal: \n" << normal << "\n";
		*/

		EigenPointType point;
		point.x = x;
		point.y = y;
		point.z = z;
		point.intensity = i;
		point.normalX = normal.x();
		point.normalY = normal.y();
		point.normalZ = normal.z();
		descriptorPoints->emplace_back(point);

		// visualize
		visualization_msgs::Marker marker;
		marker.header.frame_id = refFrame;
		marker.header.stamp = cloudHeader.stamp;
		marker.ns = i;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.scale.x = 0.1; // 화살표의 굵기
		marker.scale.y = 0.2; // 화살표 머리 부분의 굵기
		marker.color.a = 1.0; 
		marker.color.r = 1.0; 
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.pose.orientation.w = 1.0; 

		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;

		geometry_msgs::Point startPoint;
		startPoint.x = x;
        startPoint.y = y;
        startPoint.z = z;

        geometry_msgs::Point endPoint;
        endPoint.x = x + normal.x();
        endPoint.y = y + normal.y();
        endPoint.z = z + normal.z();

        marker.points.push_back(startPoint);
        marker.points.push_back(endPoint);
		marker.lifetime = ros::Duration(0.1); 

		arrowArray.markers.emplace_back(marker);
	}
	arrowPub.publish(arrowArray);
}

void DynamicFeature::saveFrame()
{
	FrameData data;
	data.header = cloudHeader;
	data.pointData = *descriptorPoints;
	std::copy(curr6DOF, curr6DOF + 6, data.DOF);

	// temp
    pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
	Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();

	Eigen::Matrix3d newRotMat;
    newRotMat = Eigen::AngleAxisd(curr6DOF[5], Eigen::Vector3d::UnitZ()) *
				 Eigen::AngleAxisd(curr6DOF[4], Eigen::Vector3d::UnitY()) *
				 Eigen::AngleAxisd(curr6DOF[3], Eigen::Vector3d::UnitX());

    transMat.block<3, 3>(0, 0) = newRotMat;
    transMat.block<3, 1>(0, 3) << curr6DOF[0], curr6DOF[1], curr6DOF[2];

	pcl::transformPointCloud(*wholePoints, *pointCloud, transMat);

	publishCloud(pubTempPointCloud, pointCloud, cloudHeader.stamp, refFrame);
	//

	frameDataQueue.push_back(data);

	if(frameDataQueue.size() >= 5)
		isFrameStackUp = true;
}

void DynamicFeature::imuEncoderPropagation()
{
	FrameData preFrame = std::move(frameDataQueue.front());
	frameDataQueue.pop_front();
	FrameData postFrame = frameDataQueue.back();

    pcl::PointCloud<EigenPointType>::Ptr pointCloud1(new pcl::PointCloud<EigenPointType>(preFrame.pointData));
    pcl::PointCloud<EigenPointType>::Ptr pointCloud2(new pcl::PointCloud<EigenPointType>(postFrame.pointData));
	preDynamicPoints = pointCloud1;
	cmpDynamicPoints = pointCloud2;
}

void DynamicFeature::cloudTransform(double pose[], pcl::PointCloud<EigenPointType>::Ptr src, pcl::PointCloud<EigenPointType>::Ptr dst)
{
	Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();

	Eigen::Matrix3d newRotMat;
    newRotMat = Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
				 Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
				 Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());

    transMat.block<3, 3>(0, 0) = newRotMat;
    transMat.block<3, 1>(0, 3) << pose[0], pose[1], pose[2];

	pcl::transformPointCloud(*src, *dst, transMat);
}	

void DynamicFeature::findUnmatchedPoints()
{
	std::fill(dynamicPointArray, dynamicPointArray+channel*resolution, false);

	kdtreeDescriptor->setInputCloud(preDynamicPoints);
	std::vector<int> idx;
	std::vector<float> dist;
	for(const auto& point : cmpDynamicPoints->points){
		kdtreeDescriptor->nearestKSearch(point, 1, idx, dist);

		EigenPointType nearestPoint = preDynamicPoints->points[idx[0]];

		// temp
		double dotProduct = point.normalX * nearestPoint.normalX + point.normalY * nearestPoint.normalY + point.normalZ * nearestPoint.normalZ;

		double angle = std::acos(dotProduct) * 180.0 / M_PI;
		bool isAngleDiff = angle > 5.0 && angle < 175.0;

		//cout << dist[0] << " " << angle << "\n";

		if(dist[0] > 0.05 || isAngleDiff){
			int cID = nearestPoint.intensity;
			clusterIdx.push_back(cID);
		}
	}

	for(int& cID : clusterIdx){
		auto rangeIter = clusterContainer.equal_range(cID);
		for (auto iter = rangeIter.first; iter != rangeIter.second; ++iter){
			int startIdx = iter->second.startIdx;
			int endIdx = iter->second.endIdx;

			for(int i = startIdx; i <= endIdx; i++)
				dynamicPointArray[i] = true;
		}
	}
}

void DynamicFeature::extractStaticPoints()
{
	if(sensor == Sensor::VELODYNE){
		pcl::fromROSMsg(info.origin_cloud, *velodyneCloud);

		pcl::PointCloud<VelodynePointType> tempCloud;
		for(int i = 0; i < velodyneCloud->points.size(); i++){
			if(!dynamicPointArray[i])
				tempCloud.emplace_back(velodyneCloud->points[i]);
		}

		publishCloud(pubStaticPoints, &tempCloud, cloudHeader.stamp, refFrame);
	}
    else if(sensor == Sensor::OUSTER){
		pcl::fromROSMsg(info.origin_cloud, *ousterCloud);

		pcl::PointCloud<OusterPointType>::Ptr tempCloud(new pcl::PointCloud<OusterPointType>);
		for(int i = 0; i < ousterCloud->points.size(); i++){
			if(!dynamicPointArray[i])
				tempCloud->emplace_back(ousterCloud->points[i]);
		}

		publishCloud(pubStaticPoints, tempCloud, cloudHeader.stamp, refFrame);
		/*
		// downsampling to image
		pcl::PointCloud<OusterPointType>::Ptr dsTempCloud(new pcl::PointCloud<OusterPointType>);
		pcl::VoxelGrid<OusterPointType> dsCloud;
		dsCloud.setInputCloud(tempCloud);
		dsCloud.setLeafSize(0.1, 0.1, 0.1);
		dsCloud.filter(*dsTempCloud);

		publishCloud(pubPostPointCloud, dsTempCloud, cloudHeader.stamp, "base_link");
		*/
		// publish remove 
		pcl::PointCloud<OusterPointType>::Ptr tempCloud2(new pcl::PointCloud<OusterPointType>);
		for(int i = 0; i < ousterCloud->points.size(); i++){
			bool isP = ousterCloud->points[i].z > -0.3 && ousterCloud->points[i].z < 1;
			if(!dynamicPointArray[i] && isP)
				tempCloud2->emplace_back(ousterCloud->points[i]);
		}
		pcl::PointCloud<OusterPointType>::Ptr dsTempCloud2(new pcl::PointCloud<OusterPointType>);
		pcl::VoxelGrid<OusterPointType> dsCloud2;
		dsCloud2.setInputCloud(tempCloud2);
		dsCloud2.setLeafSize(0.1, 0.1, 0.1);
		dsCloud2.filter(*dsTempCloud2);

		publishCloud(pubPostPointCloud, dsTempCloud2, cloudHeader.stamp, "base_link");

	}
    else if(sensor == Sensor::LIVOX){
		pcl::fromROSMsg(info.origin_cloud, *livoxCloud);

		pcl::PointCloud<LivoxPointType> tempCloud;
		for(int i = 0; i < livoxCloud->points.size(); i++){
			if(!dynamicPointArray[i])
				tempCloud.emplace_back(livoxCloud->points[i]);
		}

		publishCloud(pubStaticPoints, &tempCloud, cloudHeader.stamp, refFrame);
	}
}

void DynamicFeature::publishPointCloud()
{
	publishCloud(pubPreDynamicPointCloud, preDynamicPoints, cloudHeader.stamp, "base_link");
	publishCloud(pubCmpDynamicPointCloud, cmpDynamicPoints, cloudHeader.stamp, "base_link");
	publishCloud(pubClusterPoints, clusterPoints, cloudHeader.stamp, refFrame);
	publishCloud(pubDescriptorPoints, descriptorPoints, cloudHeader.stamp, refFrame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic");

    DynamicFeature DF;

    ROS_INFO("\033[1;32m----> Dynamic Feature Started.\033[0m");
   
    ros::spin();

    return 0;
}
