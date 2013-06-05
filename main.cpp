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

#include "cognitiveSubtraction.h"
#include "icp.h"
#include "myViewer.h"
#include "shapes/rectprism.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

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
  boost::shared_ptr< PCLPointCloud > result = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	string real_points_name;
	real_points_name = "../data/dataR" + dataset + ".pcd";
	printf("Reading real input points: %s...", real_points_name.c_str());
	fflush(stdout);
	readPCD(real_points_name, real_points);
	downsample(real_points, real_points, DOWNSAMPLE_INPUT);
	printf(" ok!\n");

	string virtual_points_name;
	virtual_points_name = "../data/dataV" + dataset + ".pcd";
	printf("Reading virtual input points: %s...", virtual_points_name.c_str());
	fflush(stdout);
	readPCD(virtual_points_name, virtual_points);
	downsample(virtual_points, virtual_points, DOWNSAMPLE_VIRTUAL);
	printf(" ok!\n");

// ICP
//	ICP *icp = new ICP(real_points, virtual_points, viewer);

// Cognitive Subtraction
 	CognitiveSubtraction *cognitiveSubtraction = new CognitiveSubtraction(real_points, virtual_points, atoi(dataset.c_str()));
  result = cognitiveSubtraction->run();
  
  std::cout<<"Cognitive Subtraction result size: "<<result->size()<<std::endl;
  
  //cluster extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (result);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (result);
  ec.extract (cluster_indices);
  std::cout<<cluster_indices.size()<<std::endl;
  
  //Visalization of the point clouds without adjustments

  
  int j = 0;    
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    if(j==1)
    {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back (result->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      //std::stringstream ss;
      //ss << "cloud_cluster_" << j << ".pcd";
      //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    }
    j++;
  }
  cloud2mm(cloud_cluster);
   
     //Sintetic cylinder
//   int h = 100;
//   int radio = 60;
//   Rot3D r(0.5, 0.2, 0.2);
//   for (float angle=0; angle<2.*M_PI;  angle+=0.5)//(M_PI_2-0.01))
//   {
//     for (float y = -h/2; y<h/2; y+=2)
//     {
//       pcl::PointXYZ p;
//       p.x = radio * cos(angle);
//       p.y = y;
//       p.z = radio * sin(angle);
//       QVec v = r * QVec::vec3(p.x, p.y, p.z);
//       p.x = v(0);
//       p.y = v(1);
//       p.z = v(2);
//       cloud_cluster->push_back(p);
//     }
//   }
  
//  pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/cloud_cup.pcd", *cloud_cluster);
  
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Results"));
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_result(cloud_cluster, 0, 255, 0);
//   viewer->addPointCloud<pcl::PointXYZ> (cloud_cluster,     color_result,    "raw_input");

  
//  std::cout<<"SIZE: "<<result->size()<<std::endl;
   
   
   

   QApplication app(argc, argv);
   
   myViewer m(cloud_cluster);
   m.cylinder();
   
   app.exec();


  
//   QVec t = QVec::vec3(0.5,0.5,0.5);
//   QVec r = QVec::vec3(0,0,0);
//   QVec point = QVec::vec3(-3,0.5,-23);
//   
//   RectPrism rect(t,r,1,1,1);
//   
//   cout<<rect.distance(point)<<endl;
  
	// deletes
	//delete cognitiveSubtraction;
	//delete icp;
	
// 	  while (!viewer->wasStopped ())
//   {
//     viewer->spinOnce(100);
//     usleep(100);
//   }
	
	
	return 0;
}


