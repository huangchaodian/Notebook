#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
	icpmain ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the CloudIn data
	cloud_in->width    = 5;
	cloud_in->height   = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
		<< std::endl;
	for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
		cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		cloud_in->points[i].z << std::endl;
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;

	for (size_t i = 0; i < cloud_in->points.size (); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	std::cout << "Transformed " << cloud_in->points.size () << " data points:"
		<< std::endl;

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix (0, 0) = cos (theta);
	transformation_matrix (0, 1) = -sin (theta);
	transformation_matrix (1, 0) = sin (theta);
	transformation_matrix (1, 1) = cos (theta);
	pcl::transformPointCloud(*cloud_in,*cloud_out,transformation_matrix);

	for (size_t i = 0; i < cloud_out->points.size (); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}