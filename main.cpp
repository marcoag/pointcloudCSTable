/*
 *    Copyright (C) 2012-2013 by Luis J. Manso
 *    Robotics and Artificial Vision Laboratory (University of Extremadura)
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <QtCore>
#include <iostream>
#include "worker.h"
#include "icp.h"

void setCamera(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

int main(int argc, char* argv[])
{
	//Check for args
	if (argc != 2)
	{
	  printf("You need to provide the number of the data set to try: 1,2,3...");
	  return -1;
	}
	string dataset = argv[1];
  
	// Read point clouds from hard disk
	printf("Reading point clouds from hard disk...\n");
	boost::shared_ptr< PCLPointCloud > real_points = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	boost::shared_ptr< PCLPointCloud > virtual_points = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	string real_points_name;
	real_points_name = "data/dataR" + dataset + ".pcd";
	printf("Reading real input points: %s\n", real_points_name.c_str());
	readPCD(real_points_name, real_points);
	downsample(real_points, real_points, DOWNSAMPLE_INPUT);

	string virtual_points_name;
	virtual_points_name = "data/dataV" + dataset + ".pcd";
	printf("Reading virtual input points: %s\n", virtual_points_name.c_str());
	readPCD(virtual_points_name, virtual_points);
	downsample(virtual_points, virtual_points, DOWNSAMPLE_VIRTUAL);

  
	//Visalization of the point clouds without adjustments
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Results"));
	setCamera(viewer);
	
	int v1(0);
	viewer->createViewPort (0.0, 0.5, 0.3333, 1.0, v1);
	viewer->setBackgroundColor (1,1,1, v1);
	viewer->addText ("Input point clouds", 10, 10, 14, 0,0,0, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_real_points(real_points, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_virtual_points(virtual_points, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ> (real_points,     color_real_points,    "raw_input",   v1);
	viewer->addPointCloud<pcl::PointXYZ> (virtual_points, color_virtual_points, "raw_virtual", v1);
  
	//Outliers
	boost::shared_ptr< PCLPointCloud > outliers = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	pclGetOutliers(real_points, virtual_points, outliers, DISTANCE_THRESHOLD);

	//visalize outliers:
	int v2(0);
	viewer->createViewPort (0., 0.0, 0.3333, 0.5, v2);
	viewer->setBackgroundColor (1,1,1, v2);
	viewer->addText ("Outliers previous to adjusts", 10, 10, 14, 0,0,0, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_outliers(outliers, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ> (outliers, color_outliers, "raw_outliers", v2);

	writePCD("raw_input.pcd",  real_points);
	writePCD("raw_virtual.pcd", virtual_points);
	writePCD("raw_outliers.pcd", outliers);

	// ICP
	printf("Running \"Iterative Closest Point\"...\n");
	ICP *icp = new ICP(real_points, virtual_points, viewer);

	// Cognitive Subtraction
	printf("Running \"Point Cloud Cognitive Subtraction\" annealed particle filter...\n");

	Worker *worker = new Worker(real_points, virtual_points, viewer, atoi(dataset.c_str()));
// 	printf("Threads: %d\n", omp_get_num_threads());

	// Visualization
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce(100);
		usleep(100);
	}


	// deletes
	delete worker;
	delete icp;
	return 0;
}




void setCamera(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
#if PCL_MAJOR_VERSION == 1
	#if PCL_MINOR_VERSION <= 6
		viewer->initCameraParameters();
		viewer->camera_.pos[0] = 0;
		viewer->camera_.pos[1] = 0;
		viewer->camera_.pos[2] = -4;
		viewer->camera_.view[0] = 0;
		viewer->camera_.view[1] = -1;
		viewer->camera_.view[2] = 0;
	#else
		viewer->setCameraPosition(0,0,-4, 0,0,0, 0,1,0);
	#endif
#else
	viewer->setCameraPosition(0,0,-4, 0,0,0, 0,1,0);
#endif
	viewer->updateCamera();

}