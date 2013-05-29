#ifndef RECTPRISMFITTING_H
#define RECTPRISMFITTING_H
#include "rectprismCloudParticle.h"
#include "innermodelManager.h"
#include "shapes/vector.h"

class RectPrismFitting: public QThread
{
   
public:
  RectPrismFitting(InnerModelManager *imm);
  RectPrismFitting(InnerModelManager *imm, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToFit);
  ~RectPrismFitting();
  void sig_term();
  void run();
  //Vector V(const double& r);
  inline void setCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToFit) { cloud=cloudToFit; }
  inline double getRandom() { return (rand()%32000)/32000.0; }
  inline void setInnerModel(InnerModelManager *innermodelManager) { this->innermodelManager=innermodelManager; }
  inline bool isComputing () { return computing; }
  
private:
  
  bool computing;
  RectPrismCloudPFInputData input;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cup;
  RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> *pf;
  Vector p[10000];
  Vector d[10000];
  double t[10000];
  
  RCParticleFilter_Config c;
  InnerModelManager *innermodelManager;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr readPCLCloud(QString name);
};

#endif