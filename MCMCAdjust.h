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

#define RANDOM_SIZE 100

class MCMCAdjust
{
public:

  MCMCAdjust();
  void initialize ();
  void adapt ();
  float computeWeight(RectPrism r);

  QVec getTranslation();
  QVec getRotation();
  QVec getScale();
  
  QVec getBestTranslation();
  QVec getBestRotation();
  QVec getBestScale();
  
  static float getRandom(float var);
  void print(std::string v);
  
  inline float getWeight() { return weight; }
  
  //data
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getData () { return data; }
  void setData (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

private:

  
  void computeWeight();
  
  //initialization methods
  void estimateEigenAndCentroid(Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid);
  void initializeFromEigenValues ();
  void gypsyInitization ();
  
  //MarcovChainMontecaro
  void MarkovChainTranslation(int index);
  void MarkovChainTranslationStepOnAll();
  void MarkovChainTranslationStepOnOne();
  
  float weight;
  float bestweight;
  RectPrism r; //actual 
  RectPrism best; //best
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr data;
  

  
  //Random variance variables
  QVec varianceC;
  QVec varianceW;
  QVec varianceR;

};

#endif
