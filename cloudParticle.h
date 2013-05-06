#ifndef CLOUDPARTICLEALIGN_H
#define CLOUDPARTICLEALIGN_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include "particleFilter.h"

#include <QMat/QMatAll>

#include "cloudPFConfig.h"

using namespace RMat;


class CloudPFInputData
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moves;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
};

class CloudPFControl
{
public:
	float pitch, roll;
	float height, yaw;
	float distances[4];	
};

class CloudParticle : public RCParticleFilter_Particle<CloudPFInputData, CloudPFControl, CloudPFConfig>
{
public:
	void initialize(const CloudPFInputData &data, const CloudPFControl &control, const CloudPFConfig *config);
	void adapt(const CloudPFControl &controlBack, const CloudPFControl &controlNew, const bool noValidCandidates);
	void computeWeight(const CloudPFInputData &data);
	QMat getQMatTransformation();
	Eigen::Matrix4f getEigenTransformation();
	float getRandom(float V);

	const static QVec getVarianceT() { return varianceT; }
	const static QVec getVarianceR() { return varianceR; }
	const static void setVarianceT(QVec rr) { varianceT = rr; }
	const static void setVarianceR(QVec rr) { varianceR = rr; }

	QVec  getT()     const { return T;    }
	float getX()     const { return T(0); }
	float getY()     const { return T(1); }
	float getZ()     const { return T(2); }
	QVec  getR()     const { return R;    }
	float getPitch() const { return R(0); }
	float getYaw()   const { return R(1); }
	float getRoll()  const { return R(2); }
	
	void setValues(QVec Tf, QVec Rf)
	{
		T = Tf;
		R = Rf;
		transformation = RTMat(R(0), R(1), R(2), T);
	}


	void print(std::string v) const
	{
		printf("%s: ", v.c_str());
		printf("(%f, %f, %f)", getX(), getY(), getZ());
		printf("[%f, %f, %f]", getPitch(), getYaw(), getRoll());
		printf("\n");
	}
	
	
	double getError() { return particleError; }
private:
	double particleError;
	QVec T;
	QVec velT;
	QVec R;
	QVec velR;
	
	static QVec varianceT;
	static QVec varianceR;

	QMat transformation;

};

#endif
