#include <stdint.h>

#include <typeinfo>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "config.h"
#include "outlierExtraction.h"

#define BASIC_PERIOD 30

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

class ICP
{
public:
  ICP(boost::shared_ptr< PCLPointCloud > cloud_input, boost::shared_ptr< PCLPointCloud > cloud_target, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
};