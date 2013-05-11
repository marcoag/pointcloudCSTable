#include "icp.h"

ICP::ICP (boost::shared_ptr< PCLPointCloud > cloud_real, boost::shared_ptr< PCLPointCloud > cloud_virtual, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	printf("Running ICP...");

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations (100);
	icp.setMaxCorrespondenceDistance(0.2);
	icp.setInputCloud(cloud_virtual);
	icp.setInputTarget(cloud_real);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
// 	*cloud_output = *cloud_input;

	fflush(stdout);
	icp.align(*cloud_output);
	printf(" ok!\n");
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);
	pclGetOutliers(cloud_real, cloud_output, cloud_outliers, DISTANCE_THRESHOLD);

	//Showing results
	int v3(0);
	viewer->createViewPort (0.3333, 0.5, 0.6666, 1.0, v3);
	viewer->addText ("Point clouds after ICP", 10, 10, 14, 0,0,0, "v3 text", v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_real(cloud_real, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_output(cloud_output, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_real, color_real, "icp_target", v3);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_output, color_output, "icp_output", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_target", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_output", v3);
	
	//Showing outliers
	int v4(0);
	viewer->createViewPort (0.3333, 0.0, 0.6666, 0.5, v4);
	viewer->addText ("ICP outliers", 10, 10, 14, 0,0,0, "v4 text", v4);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_outliers(cloud_outliers, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_outliers, color_outliers, "icp_outliers", v4);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_outliers", v4);


	writePCD("icp_input.pcd",  cloud_real);
	writePCD("icp_virtual.pcd", cloud_virtual);
	writePCD("icp_outliers.pcd", cloud_outliers);
}

