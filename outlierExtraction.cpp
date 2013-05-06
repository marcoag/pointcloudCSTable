#include "outlierExtraction.h"


OutlierExtraction::OutlierExtraction(CloudPFConfig *cfg) : QThread()
{
	config = cfg;
	computing = false;
	cloud_input               = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_virtual_transformed = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_virtual             = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_outliers            = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_outliers_outside    = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_outliers_high       = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_input_sin_ol        = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);

	cInput    = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cVirtual  = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
	cOutliers = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
}


void OutlierExtraction::run()
{
	while (true)
	{
		while (computing==false)
			usleep(5000);
		printf("Computing...\n");

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
		float vT = config->varianceT;
		float vR = config->varianceR;
		double t;
double a = getmsecofday();
		for (uint i=0; i<config->iters; i++)
		{
			CloudParticle::setVarianceT(QVec::vec3(vT, vT, vT));
			CloudParticle::setVarianceR(QVec::vec3(vR, vR, vR));
			vT *= config->annealingConstant;
			vR *= config->annealingConstant;
			QVec vr = CloudParticle::getVarianceR();
			QVec vt = CloudParticle::getVarianceT();
// 			printf("===   Step: %d   ==   VarR:(%f,%f,%f) ===  VarT:(%f,%f,%f)\n", i,vr(0),vr(1),vr(2),vt(0),vt(1),vt(2));
			pf.step(input, control, false, 8);
// 			pf.getOrderedParticle(i).getT().print("T");
// 			pf.getOrderedParticle(i).getR().print("R");
		}
double b = getmsecofday();
t = b-a;

		/// Average last best particles
/*
		QVec finalT=QVec::vec3(0,0,0);
		QVec finalR=QVec::vec3(0,0,0);
		uint ps = config->particles*0.1>0?config->particles*0.1:0;
// 		if (ps>5) ps = 5;
		ps = 1;
		for (uint i=0; i<ps; ++i)
		{
			finalT += pf.getOrderedParticle(i).getT();
			finalR += pf.getOrderedParticle(i).getR();
		}
		if (ps>0)
		{
			finalT = finalT.operator/(ps);
			finalR = finalR.operator/(ps);
		}
		CloudParticle best;
		best.setValues(finalT, finalR);
		finalT.print("T");
		finalR.print("R");
*/
		CloudParticle best = pf.getBest();
// 		best.getT().print("T");
// 		best.getR().print("R");
// 		printf("exps: %d\n", exps);
		if (exps>0)
		{
			FILE *fd = fopen("results.txt", "a");
			if (fd == NULL) { printf("Can't open results.txt!\n"); exit(-1); }
			fprintf(fd, "particles:%d ", (int)config->particles);
			fprintf(fd, "varianceR:%f ", (float)config->varianceR);
			fprintf(fd, "varianceT:%f ", (float)config->varianceT);
			fprintf(fd, "annealingConstant:%f ", (float)config->annealingConstant);
			fprintf(fd, "iters:%d ", (int)config->iters);
			fprintf(fd, "downsampleInput:%f ",   (float)DOWNSAMPLE_INPUT);
			fprintf(fd, "downsampleVirtual:%f ", (float)DOWNSAMPLE_VIRTUAL);
			fprintf(fd, "optimizationMin:%f ", (float)PARTICLE_OPTIMIZATION_MIN);
			fprintf(fd, "optimizationMax:%f ", (float)PARTICLE_OPTIMIZATION_MAX);
			fprintf(fd, "distanceFunction:%d ", (int)DISTANCE_FUNCTION);
			fprintf(fd, "input:%s ",   INPUT_FILE);
			fprintf(fd, "virtual:%s ", VIRTUAL_FILE);
			fprintf(fd, "mul:%f ",    (float)MUL_SCALE_FILE);
			fprintf(fd, "error:%f ", (float)best.getError());
			fprintf(fd, "R:%f_%f_%f ", best.getR()(0), best.getR()(1), best.getR()(2) );
			fprintf(fd, "T:%f_%f_%f ", best.getT()(0), best.getT()(1), best.getT()(2) );
			fprintf(fd, "t:%f\n", t);
			fclose( fd );
			exps--;
		}
		else
		{
// 			system("killall -9 gualzru_unknown");
			exit(0);
		}

		/// Transform the input cloud to match the target using the information provided by the particle filter
		pcl::transformPointCloud(*cloud_virtual, *cloud_virtual_transformed, best.getEigenTransformation());

		/// Use such cloud to extract outliers
		pclGetOutliers(cloud_input, cloud_virtual_transformed, cloud_outliers, PARTICLE_DISTANCE_THRESHOLD);

		cm.lock();
		*cInput    = *cloud_input;
		*cVirtual  = *cloud_virtual_transformed;
		*cOutliers = *cloud_outliers;
		cm.unlock();

		write = true;
		if (write) writeMainClouds();


		/// Subtraction itself
		cloudss.clear();
		const uint min_cluster_size = 30;
		if (cloud_outliers->points.size() > min_cluster_size)
		{
			boost::shared_ptr< pcl::search::KdTree<pcl::PointXYZ> > tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud_outliers);

			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (150);
			ec.setMinClusterSize (min_cluster_size);
			ec.setMaxClusterSize (640*480);
			ec.setSearchMethod(tree);
			ec.setInputCloud(cloud_outliers);

			std::vector<pcl::PointIndices> cluster_indices;
			ec.extract(cluster_indices);

			for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
			{
				boost::shared_ptr < pcl::PointCloud<pcl::PointXYZ> > cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
				{
					cloud_cluster->points.push_back(cloud_outliers->points[*pit]);
				}
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
				if (cloud_cluster->points.size() > min_cluster_size)
				{
// 					printf("  +1  (%u)\n", (uint32_t)cloud_cluster->points.size());
					cloudss.push_back(cloud_cluster);
				}
			}
		}
		else
		{
			printf("  Too few outlier points ( <= %d )\n", min_cluster_size);
		}

		printf("OUTLIER EXTRACTION ENDS: %u clouds\n", (unsigned int)cloudss.size());
		computing = false;
		cloudss_copy = cloudss;
	}
}

void OutlierExtraction::compute(const CloudPFControl p, bool w, int expss)
{
	write = w;
	control = p;
	exps = expss;
	computing = true;
}

void readPCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1)
	{
		fprintf(stderr, "Couldn't read file %s\n", path.c_str());
		exit(-1);
	}
}

void writePCDmm2m(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

void cloud2m(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	for (uint i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x *= 0.001;
		cloud->points[i].y *= 0.001;
		cloud->points[i].z *= 0.001;
	}
}

void cloud2mm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	for (uint i=0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x *= 1000;
		cloud->points[i].y *= 1000;
		cloud->points[i].z *= 1000;
	}
}

void writePCD(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	printf("Writing: %s  width:%d height:%d points:%d\n", path.c_str(), (int)cloud->width, (int)cloud->height, (int)cloud->points.size());
	cloud->width = 1;
	cloud->height = cloud->points.size();
	static pcl::PCDWriter writer;
	if (not cloud->empty()) writer.writeASCII(path, *cloud);
}

void downsample(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &input, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &output, float grid_size)
{
	static pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
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

void pclGetOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr virtualCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outliers, double threshold)
{
	std::vector<int> pIdxSearch(1);
	std::vector<float> pDistance(1);

	outliers->points.clear();

	cloud2m(inputCloud);
	cloud2m(virtualCloud);
	threshold*=0.001;

	pcl::KdTreeFLANN<pcl::PointXYZ> kd;
	kd.setInputCloud(virtualCloud);
// 	printf("------------\n");
	for (uint i=0; i<inputCloud->points.size(); i++)
	{
// 		const pcl::PointXYZ point(inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z);
// 		printf("%f %f %f: ", inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z);
		if (kd.nearestKSearch(inputCloud->points[i], 1, pIdxSearch, pDistance) > 0)
		{
// 			printf("%f\n", pDistance[0]);
			if (pDistance[0] > threshold)
			{
				outliers->points.push_back(inputCloud->points[i]);
			}
		}
	}
	outliers->width = outliers->points.size();
	outliers->height = 1;
	outliers->is_dense = false;
	cloud2mm(inputCloud);
	cloud2mm(virtualCloud);
	cloud2mm(outliers);

	return;
// 	writePCD("target_pclout_mm.pcd", virtualCloud);
// 	writePCD("input_pclout_mm.pcd", inputCloud);

// 	cloud2m(inputCloud);
// 	cloud2m(virtualCloud);
// printf("seg: %g\n", threshold);
// 	pcl::SegmentDifferences<pcl::PointXYZ> sgmnt;
// 	sgmnt.setDistanceThreshold(threshold);
// 	sgmnt.setTargetCloud(virtualCloud);
// 	sgmnt.setInputCloud(inputCloud);
// printf("i (%d, %d)\n", inputCloud->width,               inputCloud->height);
// 	sgmnt.segment(*outliers);
// printf("i (%d, %d)\n", inputCloud->width,               inputCloud->height);
// printf("%d\n", __LINE__);
// 	cloud2mm(inputCloud);
// 	cloud2mm(virtualCloud);
// 	cloud2mm(outliers);
}

void OutlierExtraction::writeMainClouds()
{
	writePCDmm2m("o_virtual.pcd", cloud_virtual);
	writePCDmm2m("o_input.pcd", cloud_input);
	writePCDmm2m("o_transformed.pcd", cloud_virtual_transformed);
	writePCDmm2m("o_outliers.pcd", cloud_outliers);
}


void cloneCloud(boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > read, boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > write)
{
	write->width    = read->width;
	write->height   = read->height;
	write->is_dense = read->is_dense;
	write->points   = read->points;
}

void voidCloud(boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > cloud)
{
	cloud->width  = 1;
	cloud->height = 1;
	cloud->points.resize(1);
	cloud->points[0].x = 0;
	cloud->points[0].y = -100000;
	cloud->points[0].z = -100000.;
}


void cutre_clipPointCloud3D (const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cIn, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > & cOut, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cOutsideBox, const Eigen::Vector4f &coeff)
{
	cOut->points.clear();
	cOut->points.reserve(cIn->points.size());
	Eigen::Matrix4Xf points (4, cIn->points.size());
	for (register unsigned rIdx = 0; rIdx < cIn->points.size(); ++ rIdx)
	{
		points(0, rIdx) = cIn->operator[](rIdx).x;
		points(1, rIdx) = cIn->operator[](rIdx).y;
		points(2, rIdx) = cIn->operator[](rIdx).z;
		points(3, rIdx) = 1;
	}
	Eigen::VectorXf distances = coeff.transpose() * points;
	for (register unsigned rIdx = 0; rIdx < cIn->points.size(); ++ rIdx)
	{
		if (distances (rIdx, 0) >= 0)
			cOut->points.push_back (cIn->points[rIdx]);
	}
	for (register unsigned pIdx = 0; pIdx < cIn->points.size(); ++pIdx)
	{
		if ((coeff[0]*cIn->points[pIdx].x + coeff[1]*cIn->points[pIdx].y + coeff[2]*cIn->points[pIdx].z) >= -coeff[3])
			cOut->points.push_back(cIn->points[pIdx]);
		else
			cOutsideBox->points.push_back(cIn->points[pIdx]);
	}
}


