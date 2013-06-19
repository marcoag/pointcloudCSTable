#ifndef RECTPRISMCLOUDPARTICLE_H
#define RECTPRISMCLOUDPARTICLE_H

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include "shapes/rectprism.h"

#include <QMat/QMatAll>

class BruteForceMethod
{
public:

  BruteForceMethod();
  ~BruteForceMethod();
  void initialize ();
  void adapt ();
  float computeWeight();
  void initializeFromEigenValues ();
  void gypsyInitization ();
  QVec getTranslation();
  QVec getRotation();
  QVec getScale();
  static float getRandom(float var);
  void setRectPrism (RectPrism r );
  void print(std::string v);

  void estimateEigenAndCentroid(Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid);
  
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getData () { return data; }
  void setData (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

  void incTranslation(int index);
  void incWidth(int index);
  void incRotation(int index);
  
  inline RectPrism getRectPrism() { return r; }

private:
  
  float weight;
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr data;
  
  RectPrism r;
  
  //Random variance variables
  QVec varianceC;
  QVec varianceW;
  QVec varianceR;

};

#endif
