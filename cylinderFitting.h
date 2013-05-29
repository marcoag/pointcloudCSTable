#ifndef CYLINDERFITTING_H
#define CYLINDERFITTING_H
#include "cylinderCloudParticle.h"
#include "innermodelManager.h"
#include "shapes/vector.h"

class CylinderFitting: public QThread
{
   
public:
  CylinderFitting(InnerModelManager *imm);
  CylinderFitting(InnerModelManager *imm, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFit);
  ~CylinderFitting();
  void sig_term();
  void run();
  Vector V(const double& r);
  inline double getRandom() { return (rand()%32000)/32000.0; }
  inline void setInnerModel(InnerModelManager *innermodelManager) { this->innermodelManager=innermodelManager; }
  inline bool isComputing () { return computing; }
  
private:
  
  bool computing;
  CylinderCloudPFInputData input;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cup;
  RCParticleFilter<CylinderCloudPFInputData, int, CylinderCloudParticle, RCParticleFilter_Config> *pf;
  Vector p[10000];
  Vector d[10000];
  double t[10000];
  
  RCParticleFilter_Config c;
  InnerModelManager *innermodelManager;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr readPCLCloud(QString name);
};

#endif