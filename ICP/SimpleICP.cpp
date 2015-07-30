#include "SimpleICP.h"

SimpleICP::SimpleICP(void)
{
}


SimpleICP::~SimpleICP(void)
{
}


int SimpleICP::BuildKDtree(void)
{
	kdtree.setInputCloud (targetcloud);
	return 0;
}


int SimpleICP::FindNearest(void)
{
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	pcl::PointXYZ searchPoint;
	targetpointIdx.clear();
	inputpointIdx.clear();
	pointDistance.clear();
	int size=inputcloud->size();
	for (int i=0;i<size;i++)
	{
		searchPoint=inputcloud->at(i);
		kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		targetpointIdx.push_back(pointIdxNKNSearch[0]);
		inputpointIdx.push_back(i);
		pointDistance.push_back(pointNKNSquaredDistance[0]);
	}
	return 0;
}


Eigen::Matrix4f SimpleICP::EstimateTransformation(void)
{

	Eigen::Matrix4f transformation;
	pcl::PointCloud<PointT>p,q;std::vector<int>i,j;Eigen::Matrix4f  t;
	p=*inputcloud;q=*targetcloud;i=inputpointIdx,j=targetpointIdx;
	pcl::registration::TransformationEstimationSVD<PointT,PointT>e;
	e.estimateRigidTransformation(p, i, q, j, t);
	return t;
}


int SimpleICP::SetInput(pcl::PointCloud<PointT>::Ptr input)
{
	inputcloud=input;
	return 0;
}


int SimpleICP::SetTarget(pcl::PointCloud<PointT>::Ptr taget)
{
	targetcloud=taget;
	kdtree.setInputCloud(targetcloud);
	return 0;
}


int SimpleICP::align(pcl::PointCloud<PointT>&output)
{
	pcl::console::TicToc time;time.tic();
	FindNearest();
	cout<<"FindNearest "<<time.toc()<<" ms"<<endl;
	time.tic();
	transformPointCloud (output, output,EstimateTransformation());
	cout<<"Transformation"<<time.toc()<<"ms"<<endl;
	return 0;
}
