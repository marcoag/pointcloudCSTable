#include <stdint.h>

#include <typeinfo>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include "config.h"
#include "outlierExtraction.h"

#define BASIC_PERIOD 30

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

class Worker
{
public:
	Worker(boost::shared_ptr< PCLPointCloud > real_points_F, boost::shared_ptr< PCLPointCloud > virtual_points0_F)
	{
		real_points = real_points_F;
		virtual_points0 = virtual_points0_F;
		virtual_points  = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
		//iter = 0;
		//CloudPFConfig(particles, varianceR, varianceT, annealingConstant, iters=1)
		//CloudPFConfig *config = new CloudPFConfig(1, 0, 0, 0.7);
		CloudPFConfig *config = new CloudPFConfig(PARTICLES, VARIANCE_R, VARIANCE_T, ANNEALING, ITERS);
		outlierExtraction = new OutlierExtraction(config);
		outlierExtraction->start();
		updateData();

		exps = 1;
		compute();
	}

	void compute()
	{
		callOutlierExtraction();
		while(outlierExtraction->computing)
			usleep(500);
	}

private:
	boost::shared_ptr< PCLPointCloud > real_points;
	boost::shared_ptr< PCLPointCloud > virtual_points0;
	boost::shared_ptr< PCLPointCloud > virtual_points;
	CloudPFControl control;
	int exps;

	void updateData()
	{
		/// Get data from disk
		float scale = MUL_SCALE_FILE;

		QMat TT = RTMat(T_X, T_Y, T_Z, QVec::vec3(R_X, R_Y, R_Z));
		Eigen::Matrix4f tr;
		for (int col=0; col<4; ++col)
		{
			for (int row=0; row<4; ++row)
			{
				tr(row, col) = TT(row, col);
			}
		}
		pcl::transformPointCloud(*virtual_points0, *virtual_points, tr);

		for (uint i=0; i<real_points->points.size(); ++i)
		{
			real_points->points[i].x *= scale;
			real_points->points[i].y *= scale;
			real_points->points[i].z *= scale;
		}

		for (uint i=0; i<virtual_points->points.size(); ++i)
		{
			virtual_points->points[i].x *= scale;
			virtual_points->points[i].y *= scale;
			virtual_points->points[i].z *= scale;
		}

	}

	void callOutlierExtraction()
	{
		static QTime time = QTime::currentTime();
		//controlData.elapsed = time.elapsed();
		time = QTime::currentTime();

		outlierExtraction->setPointsReal(real_points);
		outlierExtraction->setPointsVirtual(virtual_points);
		outlierExtraction->compute(control, exps);
	}

	void writeOutput()
	{
			static int v=1;
			std::vector< boost::shared_ptr< PCLPointCloud > > clouds = outlierExtraction->getClouds();
			PCLPointCloud::Ptr cloud_real     = outlierExtraction->getInput();
			PCLPointCloud::Ptr cloud_virtual  = outlierExtraction->getVirtual();
			PCLPointCloud::Ptr cloud_outliers = outlierExtraction->getOutliers();
			QString s = QString::number(v);
			while(s.length() < 5) s.prepend('0');
			writePCD((s+QString("_virtual.pcd")).toStdString(),  cloud_virtual);
			writePCD((s+QString("_real.pcd")).toStdString(),     cloud_real);
			writePCD((s+QString("_outliers.pcd")).toStdString(), cloud_outliers);
			for (int i=0; i<(int)clouds.size(); ++i)
			{
				QString s2 = QString::number(i);
				while(s2.length() < 2) s2.prepend('0');
				writePCD((s+QString("_outlier_cluster_")+s2+QString(".pcd")).toStdString(), clouds[i]);
			}
			v++;
	}

	OutlierExtraction *outlierExtraction;

};
