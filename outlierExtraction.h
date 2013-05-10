#ifndef OUTLIER_EXTRACTION_H
#define OUTLIER_EXTRACTION_H

#include <sys/time.h>

#include "config.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "config.h"
#include "cloudParticle.h"

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

void cloneCloud(boost::shared_ptr< PCLPointCloud > read, boost::shared_ptr< PCLPointCloud > write);
void voidCloud(boost::shared_ptr< PCLPointCloud > cloud);

void readPCD(std::string path, PCLPointCloud::Ptr cloud);

void writePCD(std::string path, PCLPointCloud::Ptr cloud);
void writePCDmm2m(std::string path, PCLPointCloud::Ptr cloud);

void cloud2mm(PCLPointCloud::Ptr cloud);
void cloud2m(PCLPointCloud::Ptr cloud);

void downsample(const boost::shared_ptr< PCLPointCloud > &input, const boost::shared_ptr< PCLPointCloud > &output, float grid_size);

void pclGetOutliers(PCLPointCloud::Ptr inputCloud, PCLPointCloud::Ptr virtualCloud, PCLPointCloud::Ptr outliers, double threshold);


using namespace std;


class OutlierExtraction
{
public:
	OutlierExtraction(CloudPFConfig *cfg);
	void compute(CloudPFControl c, bool write = false);
	void setPointsReal(PCLPointCloud::Ptr pts)
	{
		*cloud_input = *pts;
	}
	void setPointsVirtual(PCLPointCloud::Ptr pts)
	{
		*cloud_virtual = *pts;
	}
	std::vector< boost::shared_ptr< PCLPointCloud > > getClouds()
	{
		return cloudss_copy;
	}


	PCLPointCloud::Ptr cloud_input, cloud_input0;
	PCLPointCloud::Ptr cloud_virtual, cloud_virtual0;
	PCLPointCloud::Ptr cloud_virtual_transformed;
	PCLPointCloud::Ptr cloud_outliers;

	PCLPointCloud::Ptr getInput()    { return cloud_input; }
	PCLPointCloud::Ptr getVirtual()  { return cloud_virtual_transformed; }
	PCLPointCloud::Ptr getOutliers() { return cloud_outliers; }

private:
	CloudPFControl control;
	CloudPFConfig *config;

private:
	std::vector< boost::shared_ptr< PCLPointCloud > > cloudss, cloudss_copy;

	void writeMainClouds();
	double getmsecofday()
	{
		static timeval tv;
		gettimeofday(&tv, NULL);
		return double(tv.tv_usec)/double(1000) + double(tv.tv_sec)*double(1000.);
	}

};


#endif
