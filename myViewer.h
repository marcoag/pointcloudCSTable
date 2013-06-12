#ifndef MYVIWER_H
#define MYVIWER_H

#include <QWidget>

#include <pcl/io/openni_grabber.h>

#include <osgviewer/osgview.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <string>

#include "innermodelManager.h"
#include "cylinderFitting.h"
#include "rectprismFitting.h"

using namespace std;

//typedef pcl::PointXYZRGBA PointT;

class myViewer: public QWidget
{
Q_OBJECT
public:
  myViewer();
  myViewer(pcl::PointCloud<PointT>::Ptr cloudToFit);
  ~myViewer();
  void cube();
//   void cylinder();
//   void setXmlPath(string xml);
  
  void resizeEvent(QResizeEvent * event);
  void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud);

public slots:
  void runCylinder();
  void runRectPrism();
private:
  
  pcl::Grabber* interface;
  pcl::PointCloud<PointT>::Ptr cloudToFit;
  pcl::PointCloud<PointT>::Ptr cloudToShow;
  QTimer timer;
  OsgView *world3D;
  InnerModelManager *innerModelManager;
  CylinderFitting *cylinderFitting;
  RectPrismFitting *rectprismFitting;
  QMutex innermodelMutex, cloudToFitMutex;
  bool first_cloud;
};

#endif