#ifndef OUTLIER_EXTRACTION_H
#define OUTLIER_EXTRACTION_H

#include <sys/time.h>

#include "config.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "config.h"
#include "cloudParticle.h"

void cloneCloud(boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > read, boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > write);
void voidCloud(boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud);
void cutre_clipPointCloud3D (const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cIn, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > & cOut, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cOutsideBox, const Eigen::Vector4f &coeff);

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

class OutlierExtraction
{
public:
	OutlierExtraction(CloudPFConfig *cfg);
	void compute(CloudPFControl c, bool write = false, int iters=1);
	void setPointsReal(pcl::PointCloud<pcl::PointXYZ>::Ptr pts)
	{
		cloud_input = pts;
	}
	void setPointsVirtual(pcl::PointCloud<pcl::PointXYZ>::Ptr pts)
	{
		cloud_virtual = pts;
	}
	std::vector< boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > > getClouds()
	{
		return cloudss_copy;
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_virtual;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_virtual_transformed;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cInput, cVirtual, cOutliers;

	pcl::PointCloud<pcl::PointXYZ>::Ptr getInput()    { return cInput; }
	pcl::PointCloud<pcl::PointXYZ>::Ptr getVirtual()  { return cVirtual; }
	pcl::PointCloud<pcl::PointXYZ>::Ptr getOutliers() { return cOutliers; }

private:
	CloudPFControl control;
	bool write;
	int32_t exps;

	CloudPFConfig *config;

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_sin_ol;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud_outliers_outside;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud_outliers_high;
	std::vector< boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > > cloudss, cloudss_copy;
	std::vector< boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > > cloudsShow;

	void writeMainClouds();
	double getmsecofday()
	{
		static timeval tv;
		gettimeofday(&tv, NULL);
		return double(tv.tv_usec)/double(1000) + double(tv.tv_sec)*double(1000.);
	}

};

void setCloudFromPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PCLPointCloud::Ptr points);
void readPCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void writePCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void writePCDmm2m(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void cloud2mm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void cloud2m(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void downsample(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &input, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &output, float grid_size);
void pclGetOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr virtualCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outliers, double threshold);

#endif
