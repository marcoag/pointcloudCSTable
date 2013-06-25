#ifndef MYVIWER_H
#define MYVIWER_H

#include <QWidget>

#include <pcl/io/openni_grabber.h>

#include <osgviewer/osgview.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <string>

#include "innermodelManager.h"
#include "rectprismFitting.h"

using namespace std;

class myViewer: public QWidget
{
Q_OBJECT
public:
  myViewer();
  ~myViewer();
  void cube();
  
  void resizeEvent(QResizeEvent * event);

public slots:
  void runRectPrism();
private:
  
  //Mutex for shared pointers
  boost::shared_ptr<QMutex> innerModelMutex;
  
  //Fitting
  RectPrismFitting *rectprismFitting;
  QTimer timer;
  
  //Visualization management
  OsgView *world3D;
  InnerModelManager *innerModelManager;
  
};

#endif