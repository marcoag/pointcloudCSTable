#ifndef RECTPRISMCLOUDPARTICLE_H
#define RECTPRISMCLOUDPARTICLE_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "particleFiltering/particleFilter.h"
#include "shapes/rectprism.h"

#include <QMat/QMatAll>

class RectPrismCloudPFInputData
{
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target;
};

class RectPrismCloudParticle : public RCParticleFilter_Particle<RectPrismCloudPFInputData, int, RCParticleFilter_Config>
{
public:

  RectPrismCloudParticle();
  ~RectPrismCloudParticle();
  void initialize(const RectPrismCloudPFInputData &data, const int &control, const RCParticleFilter_Config *config);
  void adapt(const int &controlBack, const int &controlNew, const bool noValidCandidates);
  void computeWeight(const RectPrismCloudPFInputData &data);
  void initializeFromEigenValues(const RectPrismCloudPFInputData &data);
  void gypsyInitization(const RectPrismCloudPFInputData &data);
  QVec getTranslation() const;
  QVec getRotation();
  QVec getScale() const;
  float getRandom(float var);
  void setRectPrism (RectPrism r );

  void estimateEigenAndCentroid(const RectPrismCloudPFInputData &data, Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid);

  
  inline RectPrism getRectPrism() { return c; };

  void print(std::string v)
  {
    printf("%s: \n", v.c_str());
    printf("RectPrism: A(%f,%f,%f), B(%f,%f,%f), r=%f, Weight: %f\n", c.getA().getX() , c.getA().getY() ,c.getA().getZ() , c.getB().getX() ,c.getB().getY() ,c.getB().getZ(), c.getR(), weight);
  }   
private:
  RectPrism r;
  
  //Random variance variables
  QVec varianceA;
  QVec varianceB;
  float varianceR;



};

#endif
