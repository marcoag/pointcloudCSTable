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
  void cube();
  void cylinder();
//   void setXmlPath(string xml);
  inline string getXmlPath() { return xmlLocation; }
  
  void resizeEvent(QResizeEvent * event);

public slots:
  void run();
private:
  
  QTimer timer;
  string xmlLocation;
  OsgView *world3D;
  InnerModelManager *innerModelManager;
  CylinderFitting *cylinderFitting;
  RectPrismFitting *rectprismFitting;
};

#endif