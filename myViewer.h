#ifndef MYVIWER_H
#define MYVIWER_H

#include <QWidget>

#include <osgviewer/osgview.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <string>

#include "innermodelManager.h"
#include "cylinderFitting.h"
#include "rectprismFitting.h"

using namespace std;

class myViewer: public QWidget
{
Q_OBJECT
public:
  myViewer();
  myViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFit, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToShow);
  void cube();
  void cylinder();
//   void setXmlPath(string xml);
  inline string getXmlPath() { return xmlLocation; }
  
  void resizeEvent(QResizeEvent * event);

public slots:
  void runCylinder();
  void runRectPrism();
private:
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFit;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToShow;
  bool cloudGiven;
  QTimer timer;
  string xmlLocation;
  OsgView *world3D;
  InnerModelManager *innerModelManager;
  CylinderFitting *cylinderFitting;
  RectPrismFitting *rectprismFitting;
};

#endif