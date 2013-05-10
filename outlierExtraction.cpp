#include "outlierExtraction.h"


OutlierExtraction::OutlierExtraction(CloudPFConfig *cfg)
{
	config = cfg;
	cloud_input               = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	cloud_input0              = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	cloud_virtual0            = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	cloud_virtual_transformed = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	cloud_virtual             = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	cloud_outliers            = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
}

void OutlierExtraction::compute(const CloudPFControl p, bool w)
{
	control = p;
	*cloud_input0 = *cloud_input;
	*cloud_virtual0 = *cloud_virtual;
	downsample(cloud_input, cloud_input, DOWNSAMPLE_INPUT);
	downsample(cloud_virtual, cloud_virtual, DOWNSAMPLE_VIRTUAL);
	/// Set the point cloud 'cloud_virtual' from the virtual point set 'virtualPoints'.
	/// Particle filter
	CloudPFInputData input;
	input.cloud_static = cloud_virtual;
	input.cloud_moves  = cloud_input;
	input.kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	input.kdtree->setInputCloud(input.cloud_static);
	RCParticleFilter<CloudPFInputData, CloudPFControl, CloudParticle, CloudPFConfig> pf(config, input, control);
	QVec vT = config->varianceT;
	QVec vR = config->varianceR;

	for (uint i=0; i<config->iters; i++)
	{
		CloudParticle::setVarianceT(QVec::vec3(vT(0), vT(1), vT(2)));
		CloudParticle::setVarianceR(QVec::vec3(vR(0), vR(1), vR(2)));
		vT.operator*(config->annealingConstant);
		vR.operator*(config->annealingConstant);
		pf.step(input, control, true);
	}

	CloudParticle best = pf.getBest();
	

	/// Transform the input cloud to match the target using the information provided by the particle filter
	pcl::transformPointCloud(*cloud_virtual0, *cloud_virtual_transformed, best.getEigenTransformation());

	/// Use such cloud to extract outliers
	pclGetOutliers(cloud_input0, cloud_virtual_transformed, cloud_outliers, DISTANCE_THRESHOLD);

// 	/// Subtraction itself
// 	cloudss.clear();
// 	const uint min_cluster_size = 30;
// 	if (cloud_outliers->points.size() > min_cluster_size)
// 	{
// 		boost::shared_ptr< pcl::search::KdTree<pcl::PointXYZ> > tree(new pcl::search::KdTree<pcl::PointXYZ>);
// 		tree->setInputCloud(cloud_outliers);
// 
// 		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
// 		ec.setClusterTolerance (150);
// 		ec.setMinClusterSize (min_cluster_size);
// 		ec.setMaxClusterSize (640*480);
// 		ec.setSearchMethod(tree);
// 		ec.setInputCloud(cloud_outliers);
// 
// 		std::vector<pcl::PointIndices> cluster_indices;
// 		ec.extract(cluster_indices);
// 
// 		for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
// 		{
// 			boost::shared_ptr < PCLPointCloud > cloud_cluster(new PCLPointCloud);
// 			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
// 			{
// 				cloud_cluster->points.push_back(cloud_outliers->points[*pit]);
// 			}
// 			cloud_cluster->width = cloud_cluster->points.size();
// 			cloud_cluster->height = 1;
// 			cloud_cluster->is_dense = true;
// 			if (cloud_cluster->points.size() > min_cluster_size)
// 			{
// 				cloudss.push_back(cloud_cluster);
// 			}
// 		}
// 	}
// 	else
// 	{
// 		printf("  Too few outlier points ( <= %d )\n", min_cluster_size);
// 	}
// 	printf("OUTLIER EXTRACTION ENDS: %u clouds\n", (unsigned int)cloudss.size());
// 	cloudss_copy = cloudss;

}

void readPCD(std::string path, PCLPointCloud::Ptr cloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1)
	{
		fprintf(stderr, "Couldn't read file %s\n", path.c_str());
		exit(-1);
	}
}

void writePCDmm2m(std::string path, PCLPointCloud::Ptr cloud)
{
	for (uint i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x *= 0.001;
		cloud->points[i].y *= 0.001;
		cloud->points[i].z *= 0.001;
	}
	writePCD(path, cloud);
	for (uint i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x *= 1000;
		cloud->points[i].y *= 1000;
		cloud->points[i].z *= 1000;
	}
}

void cloud2m(PCLPointCloud::Ptr cloud)
{
	for (uint i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x *= 0.001;
		cloud->points[i].y *= 0.001;
		cloud->points[i].z *= 0.001;
	}
}

void cloud2mm(PCLPointCloud::Ptr cloud)
{
	for (uint i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x *= 1000;
		cloud->points[i].y *= 1000;
		cloud->points[i].z *= 1000;
	}
}

void writePCD(std::string path, PCLPointCloud::Ptr cloud)
{
	printf("Writing: %s  width:%d height:%d points:%d\n", path.c_str(), (int)cloud->width, (int)cloud->height, (int)cloud->points.size());
	cloud->width = 1;
	cloud->height = cloud->points.size();
	static pcl::PCDWriter writer;
	if (not cloud->empty()) writer.writeASCII(path, *cloud);
}

void downsample(const boost::shared_ptr< PCLPointCloud > &input, const boost::shared_ptr< PCLPointCloud > &output, float grid_size)
{
	static PCLPointCloud cloud_downsampled;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setLeafSize(grid_size, grid_size, grid_size);
	if (input.get() == output.get())
	{
		sor.setInputCloud(input);
		sor.filter(cloud_downsampled);
		*output = cloud_downsampled;
	}
	else
	{
		sor.setInputCloud(input);
		sor.filter(*output);
	}
}

void pclGetOutliers(PCLPointCloud::Ptr inputCloud, PCLPointCloud::Ptr virtualCloud, PCLPointCloud::Ptr outliers, double threshold)
{
	std::vector<int> pIdxSearch(1);
	std::vector<float> pDistance(1);

	outliers->points.clear();

	pcl::KdTreeFLANN<pcl::PointXYZ> kd;
	kd.setInputCloud(virtualCloud);
	for (uint i=0; i<inputCloud->points.size(); i++)
	{
		if (kd.nearestKSearch(inputCloud->points[i], 1, pIdxSearch, pDistance) > 0)
		{
			if (pDistance[0] > threshold)
			{
				outliers->points.push_back(inputCloud->points[i]);
			}
		}
	}
	outliers->width = outliers->points.size();
	outliers->height = 1;
	outliers->is_dense = false;

	return;
}

void cloneCloud(boost::shared_ptr< PCLPointCloud > read, boost::shared_ptr< PCLPointCloud > write)
{
	write->width    = read->width;
	write->height   = read->height;
	write->is_dense = read->is_dense;
	write->points   = read->points;
}

void voidCloud(boost::shared_ptr< PCLPointCloud > cloud)
{
	cloud->width  = 1;
	cloud->height = 1;
	cloud->points.resize(1);
	cloud->points[0].x = 0;
	cloud->points[0].y = -100000;
	cloud->points[0].z = -100000.;
}


