#ifndef COGNITIVESUBTRACTION_H
#define COGNITIVESUBTRACTION_H

#include <stdint.h>

#include <typeinfo>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "config.h"
#include "outlierExtraction.h"

#define BASIC_PERIOD 30

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

class CognitiveSubtraction
{
public:
	CognitiveSubtraction(boost::shared_ptr< PCLPointCloud > real_points_F, boost::shared_ptr< PCLPointCloud > virtual_points_F, int dataset);
  PCLPointCloud::Ptr run();

private:
	boost::shared_ptr< PCLPointCloud > real_points;
	boost::shared_ptr< PCLPointCloud > virtual_points;
	OutlierExtraction *outlierExtraction;
	CloudPFControl control;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  
};

#endif