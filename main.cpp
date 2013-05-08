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

int main(int argc, char* argv[])
{
	// Read point clouds from hard disk
	printf("Reading point clouds from hard disk...\n");
	boost::shared_ptr< PCLPointCloud > real_points = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	boost::shared_ptr< PCLPointCloud > virtual_points0 = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	printf("Reading real input points: %s\n", argv[1]);
	readPCD(argv[1], real_points);
	printf("Reading virtual input points: %s\n", argv[2]);
	readPCD(argv[2], virtual_points0);
 
  std::cout<<"number of real: "<<real_points->size()<<std::endl;
  std::cout<<"number of virtual: "<<virtual_points0->size()<<std::endl;
  
	//Visalization of the point clouds without adjustments
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Results"));
	int v1(0);
	viewer->createViewPort (0.0, 0.6666, 0.5, 1.0, v1);
	viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addText ("Input point clouds", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_real_points(real_points, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_virtual_points(virtual_points0, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ> (real_points,     color_real_points,    "raw_input",   v1);
	viewer->addPointCloud<pcl::PointXYZ> (virtual_points0, color_virtual_points, "raw_virtual", v1);
  
	//Outliers
	boost::shared_ptr< PCLPointCloud > outliers = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
  pcl::SegmentDifferences<pcl::PointXYZ> sgmnt;
  sgmnt.setDistanceThreshold(0.1);
  sgmnt.setTargetCloud(virtual_points0);
  sgmnt.setInputCloud(real_points);
  sgmnt.segment(*outliers);

	//visalize outliers:
	int v2(0);
	viewer->createViewPort (0.5, 0.6666, 1.0, 1.0, v2);
	viewer->addText ("Outliers previous to adjusts", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_outliers(outliers, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ> (outliers, color_outliers, "raw_outliers", v2);


	// ICP
	ICP *icp = new ICP(real_points, virtual_points0, viewer);

	// Cognitive Subtraction
	Worker *worker = new Worker(real_points, virtual_points0, viewer);

	// Visualization
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce(100);
		usleep(100);
	}


	// deletes
// 	delete worker;
// 	delete icp;
	return 0;
}



