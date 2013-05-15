#include "myViewer.h"

// myViewer::myViewer()
// {
// }

myViewer::myViewer(string xml) : QWidget()
{
  xmlLocation=xml;
   InnerModel *innerModel = new InnerModel(xmlLocation);
  
  QGLFormat fmt;
  fmt.setDoubleBuffer(true);
  QGLFormat::setDefaultFormat(fmt);
  world3D = new OsgView(this);
  world3D->init();
//  world3D->addXYZAxisOnNode(world3D->getRootGroup(), 2, 0.2, osg::Vec4(1,0,0,0.5));

  InnerModelViewer *imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup());
  world3D->getRootGroup()->addChild(imv);
  world3D->show();
  world3D->setHomePosition(QVecToOSGVec(QVec::vec3(0,2000,-2000)), QVecToOSGVec(QVec::vec3(0,0,6000)), QVecToOSGVec(QVec::vec3(0,8000,-2000)), false);
  this->show();

  
  innerModelManager = new InnerModelManager(innerModel, imv);

  cylinderFitting = new CylinderFitting(innerModelManager);
  
  connect (&timer, SIGNAL(timeout()),this,SLOT(run()));
  timer.start(10);
}

void myViewer::run()
{
  if(cylinderFitting->isComputing())
  {
    world3D->update();
  }
  else
  {
    cylinderFitting->start();
    world3D->update();
  }
}

void myViewer::resizeEvent(QResizeEvent * event)
{
  world3D->autoResize();
}