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

//#define LIVE

typedef pcl::PointXYZRGBA PointT;

class RectPrismFitting: public QThread
{
   
public:
  RectPrismFitting(InnerModelManager *imm);
  RectPrismFitting(InnerModelManager *imm, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToFit);
  ~RectPrismFitting();
  void sig_term();
  void run();
  //Vector V(const double& r);
  void setCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToFit);
  inline double getRandom() { return (rand()%32000)/32000.0; }
  inline void setInnerModel(InnerModelManager *innermodelManager) { this->innermodelManager=innermodelManager; }
  inline bool isComputing () { return computing; }
  static void cloud2m(pcl::PointCloud<PointT>::Ptr cloud);
  static void cloud2mm(pcl::PointCloud<PointT>::Ptr cloud);
  
private:
  
  pcl::PointCloud<PointT>::Ptr ransacAndEuclideanCluster(float ransacDistanceThreshold, float clusterTolerance);
  pcl::PointCloud<PointT>::Ptr getSinteticCube();
  
  bool computing;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cup;
  BruteForceMethod bruteforce;
  Vector p[10000];
  Vector d[10000];
  double t[10000];
  
  InnerModelManager *innermodelManager;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr readPCLCloud(QString name);
  
  //cloud extract
  pcl::PointIndices::Ptr inliers_plane;
  std::vector<int> inliers;
  pcl::ExtractIndices<PointT> extract;
  pcl::PointCloud<PointT>::Ptr final_;
};

#endif