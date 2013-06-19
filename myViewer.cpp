#include "myViewer.h"

myViewer::myViewer() : QWidget(),
cloudToFit(new pcl::PointCloud<PointT>())
 ,cloudToShow(new pcl::PointCloud<PointT>())
 ,first_cloud(false)
{
  interface = new pcl::OpenNIGrabber();
}

myViewer::myViewer(pcl::PointCloud<PointT>::Ptr cloudToFit) : QWidget(),
cloudToFit(new pcl::PointCloud<PointT>())
 ,cloudToShow(new pcl::PointCloud<PointT>())
 ,first_cloud(true)
{
  this->cloudToFit = cloudToFit;
}

myViewer::~myViewer()
{
  interface->stop();
}

void myViewer::cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud)
{


 // *cloudToShow = *cloud;
  
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloudToShow);
  
  RectPrismFitting::cloud2mm(cloudToShow);
  
  cloudToFitMutex.tryLock();
  *cloudToFit = *cloudToShow;
  
  if (!first_cloud)
  {
    rectprismFitting = new RectPrismFitting(innerModelManager,cloudToFit);
    connect (&timer, SIGNAL(timeout()),this,SLOT(runRectPrism()));
    timer.start(10);
    first_cloud=true;
  }
  cloudToFitMutex.unlock();
  
  //update innermodel
  innermodelMutex.lock();
  //innerModelManager->setPointCloudData("cup_cloud", cloudToShow);
  innermodelMutex.unlock();
  
  world3D->update();
}

void myViewer::cube()
{
  
  InnerModel *innerModel = new InnerModel("../scenarios/cubePointCloud.xml");
  
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
  
  //create callback for kinect
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
    boost::bind (&myViewer::cloud_cb_, this, _1);
  //register and start it
  interface->registerCallback (f);
  interface->start ();
  
//   if(!cloudGiven)
//     rectprismFitting = new RectPrismFitting(innerModelManager);
//   else

}

// void myViewer::cylinder()
// {
//   InnerModel *innerModel = new InnerModel("../scenarios/genericPointCloud.xml");
//   
//   QGLFormat fmt;
//   fmt.setDoubleBuffer(true);
//   QGLFormat::setDefaultFormat(fmt);
//   world3D = new OsgView(this);
//   world3D->init();
// //  world3D->addXYZAxisOnNode(world3D->getRootGroup(), 2, 0.2, osg::Vec4(1,0,0,0.5));
// 
//   InnerModelViewer *imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup());
//   world3D->getRootGroup()->addChild(imv);
//   world3D->show();
//   world3D->setHomePosition(QVecToOSGVec(QVec::vec3(0,2000,-2000)), QVecToOSGVec(QVec::vec3(0,0,6000)), QVecToOSGVec(QVec::vec3(0,8000,-2000)), false);
//   this->show();
// 
//   
//   innerModelManager = new InnerModelManager(innerModel, imv);
// 
//   if(!cloudGiven)
//     cylinderFitting = new CylinderFitting(innerModelManager);
//   else
//     cylinderFitting = new CylinderFitting(innerModelManager,cloudToFit);
//   
//   connect (&timer, SIGNAL(timeout()),this,SLOT(runCylinder()));
//   timer.start(10);
// }
void myViewer::runRectPrism()
{
  
  if(rectprismFitting->isComputing())
  {
    world3D->update();
  }
  else
  {
    cloudToFitMutex.lock();
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    *cloud = *cloudToFit;
    cloudToFitMutex.unlock();
    
    rectprismFitting->setCloud(cloud);
    rectprismFitting->start();
    world3D->update();
  }
}

// void myViewer::runCylinder()
// {
//   if(cylinderFitting->isComputing())
//   {
//     world3D->update();
//   }
//   else
//   {
//     cylinderFitting->start();
//     world3D->update();
//   }
//   getchar();
// }

void myViewer::resizeEvent(QResizeEvent * event)
{
  world3D->autoResize();
}