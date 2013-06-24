#ifndef RECTPRISMFITTING_H
#define RECTPRISMFITTING_H

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include "innermodelManager.h"
#include "bruteForceMethod.h"

#include <Eigen/Geometry>

typedef pcl::PointXYZRGBA PointT;

class RectPrismFitting: public QThread
{
   
public:
  RectPrismFitting(InnerModelManager *imm, boost::shared_ptr<QMutex> immMutex);
  void run();

  inline bool isComputing () { return computing; }
  
private:
  void makeSinteticCube();

  bool computing;

  
  InnerModelManager *innermodelManager;
  boost::shared_ptr<QMutex> innerModelMutex;
  
  BruteForceMethod *bruteForceMethod;
  pcl::PointCloud<PointT>::Ptr cloudToFit;
  
};

#endif