#include "myViewer.h"

myViewer::myViewer(): QWidget(),
innerModelMutex(new QMutex())
{
  
}

myViewer::~myViewer()
{

}

void myViewer::cube()
{
  
  InnerModel *innerModel = new InnerModel("../scenarios/cubePointCloud.xml");
  
  QGLFormat fmt;
  fmt.setDoubleBuffer(true);
  QGLFormat::setDefaultFormat(fmt);
  world3D = new OsgView(this);
  world3D->init();
  
  InnerModelViewer *imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup());
    
  world3D->getRootGroup()->addChild(imv);
  world3D->show();
  world3D->setHomePosition(QVecToOSGVec(QVec::vec3(0,2000,-2000)), QVecToOSGVec(QVec::vec3(0,0,6000)), QVecToOSGVec(QVec::vec3(0,8000,-2000)), false);
  this->show();
  
  //initialize all!
  innerModelManager = new InnerModelManager(innerModel, imv);
  
  rectprismFitting = new RectPrismFitting(innerModelManager, innerModelMutex);
  connect (&timer, SIGNAL(timeout()),this,SLOT(runRectPrism()));
  timer.start(10);
}

void myViewer::runRectPrism()
{
  
  if(rectprismFitting->isComputing())
  {
    world3D->update();
  }
  else
  {
    rectprismFitting->start();
    world3D->update();
  }
}


void myViewer::resizeEvent(QResizeEvent * event)
{
  world3D->autoResize();
}