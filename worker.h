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

class Worker
{
public:
	Worker(boost::shared_ptr< PCLPointCloud > real_points_F, boost::shared_ptr< PCLPointCloud > virtual_points_F,  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
	{
		real_points = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
		virtual_points = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);

		*real_points = *real_points_F;
		*virtual_points = *virtual_points_F;

		CloudPFConfig *config = new CloudPFConfig(PARTICLES, vT_X, vT_Y, vT_Z, vR_X, vR_Y, vR_Z, ANNEALING, ITERS);
		outlierExtraction = new OutlierExtraction(config);

		compute();
		std::cout<<"number of outliers: "<<outlierExtraction->getOutliers()->size()<<std::endl;

		//show results:
		int v5(0);
		viewer->createViewPort (0., 0.0, 0.5, 0.3333, v5);
		viewer->setBackgroundColor (0, 0, 0, v5);
		viewer->addText ("Cognitive substraction adjust", 10, 10, "v5 text", v5);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_real_pointsv5(real_points, 255, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_virtual_pointsv5(virtual_points, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ> (real_points, color_real_pointsv5, "real_pointsv5", v5);
		viewer->addPointCloud<pcl::PointXYZ> (virtual_points, color_virtual_pointsv5, "virtual_pointsv5", v5);

		//Show outliers
		int v6(0);
		viewer->createViewPort (0.5, 0.0, 1.0, 0.3333, v6);
		viewer->setBackgroundColor (0, 0, 0, v6);
		viewer->addText ("Cognitive substraction outliers", 10, 10, "v6 text", v6);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_outliersv6(outlierExtraction->getOutliers(), 0, 0, 255);
		viewer->addPointCloud<pcl::PointXYZ> (outlierExtraction->getOutliers(), color_outliersv6, "cloud_outliersv6", v6);   
	}

	void compute()
	{
		outlierExtraction->setPointsReal(real_points);
		outlierExtraction->setPointsVirtual(virtual_points);
		outlierExtraction->compute(control);
	}

private:
	boost::shared_ptr< PCLPointCloud > real_points;
	boost::shared_ptr< PCLPointCloud > virtual_points;
	OutlierExtraction *outlierExtraction;
	CloudPFControl control;
};
