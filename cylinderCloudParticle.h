#ifndef CYLINDERCLOUDPARTICLE_H
#define CYLINDERCLOUDPARTICLE_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "particleFiltering/particleFilter.h"
#include "shapes/cylinder.h"

#include <QMat/QMatAll>

class CylinderCloudPFInputData
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target;
};

class CylinderCloudParticle : public RCParticleFilter_Particle<CylinderCloudPFInputData, int, RCParticleFilter_Config>
{
public:

  CylinderCloudParticle();
  ~CylinderCloudParticle();
	void initialize(const CylinderCloudPFInputData &data, const int &control, const RCParticleFilter_Config *config);
	void adapt(const int &controlBack, const int &controlNew, const bool noValidCandidates);
	void computeWeight(const CylinderCloudPFInputData &data);
  void initializeFromEigenValues(const CylinderCloudPFInputData &data);
  void gypsyInitization(const CylinderCloudPFInputData &data);
  QVec getTranslation() const;
  QVec getRotation();
  QVec getScale() const;
  float getRandom(float var);
  inline void setCylinder (Cylinder cyl ) {std::cout<<"set"<<cyl.getR()<<std::endl;this->c.setValues(cyl.getA(), cyl.getB(), cyl.getR());}

  void estimateEigenAndCentroid(const CylinderCloudPFInputData &data, Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid);

  
  inline Cylinder getCylinder() { return c; };

	void print(std::string v)
	{
    printf("%s: \n", v.c_str());
    printf("Cylinder: A(%f,%f,%f), B(%f,%f,%f), r=%f, Weight: %f\n", c.getA().getX() , c.getA().getY() ,c.getA().getZ() , c.getB().getX() ,c.getB().getY() ,c.getB().getZ(), c.getR(), weight);
	}   
private:
  Cylinder c;
  
  //Random variance variables
  QVec varianceA;
  QVec varianceB;
  float varianceR;



};

#endif
