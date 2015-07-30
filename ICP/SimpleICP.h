#pragma once
#include <iostream>
#include <string>

//#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
//#include <pcl/tracking/tracking.h>
//#include <pcl/registration/transformation_estimation.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class SimpleICP
{
public:
	SimpleICP(void);
	~SimpleICP(void);
	pcl::PointCloud<PointT>::Ptr inputcloud;
	pcl::PointCloud<PointT>::Ptr targetcloud;

	pcl::KdTreeFLANN<PointT> kdtree;
	std::vector<int> targetpointIdx;
	std::vector<int> inputpointIdx;
	std::vector<float> pointDistance;
	//pcl::registration::TransformationEstimationSVD::estimateRigidTransformation() transformation_estimation_;
	int BuildKDtree(void);
	int FindNearest(void);
	Eigen::Matrix4f EstimateTransformation(void);
	int SetInput(pcl::PointCloud<PointT>::Ptr input);
	int SetTarget(pcl::PointCloud<PointT>::Ptr taget);
	int align(pcl::PointCloud<PointT>&output);
};

