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
// 	CognitiveSubtraction *cognitiveSubtraction = new CognitiveSubtraction(real_points, virtual_points, atoi(dataset.c_str()));
//   result = cognitiveSubtraction->run();
  
//  std::cout<<"SIZE: "<<result->size()<<std::endl;

   QApplication app(argc, argv);
//   
   myViewer m;
   m.cube();
   app.exec();
  
  vector<double> t;
  t.push_back(0);
  t.push_back(0);
  t.push_back(0);
  vector<double> r;
  r.push_back(0);
  r.push_back(0);
  r.push_back(0);
  //point
  vector<double> p;
  p.push_back(1);
  p.push_back(1);
  p.push_back(1);
  
  RectPrism rect(t,r,2,2,2);
  
  cout<<rect.distance(p)<<endl;
  
	// deletes
	//delete cognitiveSubtraction;
	//delete icp;
	return 0;
}


