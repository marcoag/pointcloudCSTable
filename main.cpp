#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <QWidget>

#include <osgviewer/osgview.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include "myViewer.h"
#include "innermodelManager.h"
#include "rectprismFitting.h"


// class SimpleOpenNIViewer: public QWidget
// {
//   public:
//     SimpleOpenNIViewer () : QWidget() 
//     ,final_ (new pcl::PointCloud<PointT>)
//     ,inliers_plane (new pcl::PointIndices)
//     {
// 
//       innerModel = new InnerModel("../scenarios/cubePointCloud.xml");
//       
//       QGLFormat fmt;
//       fmt.setDoubleBuffer(true);
//       QGLFormat::setDefaultFormat(fmt);
//       world3D = new OsgView(this);
//       world3D->init();
//     //  world3D->addXYZAxisOnNode(world3D->getRootGroup(), 2, 0.2, osg::Vec4(1,0,0,0.5));
//       
//       imv = new InnerModelViewer(innerModel, "root", world3D->getRootGroup());
//         
//       world3D->getRootGroup()->addChild(imv);
//       world3D->show();
//       world3D->setHomePosition(QVecToOSGVec(QVec::vec3(0,2000,-2000)), QVecToOSGVec(QVec::vec3(0,0,6000)), QVecToOSGVec(QVec::vec3(0,8000,-2000)), false);
//       this->show();
// 
//       innerModelManager = new InnerModelManager(innerModel, imv);
//       
//       rectprismFitting = new RectPrismFitting(innerModelManager);
//       
//     }
// 
//     void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud)
//     {
//       cout<<"aaaaaaaaaaaaaaaaaaaaaa"<<endl;
// 
// //       pcl::SampleConsensusModelPlane<PointT>::Ptr
// //         model_s(new pcl::SampleConsensusModelPlane<PointT> (cloud));
// //       //Ransac
// //       pcl::RandomSampleConsensus<PointT> ransac (model_s);
// //       ransac.setDistanceThreshold (.01);
// //       ransac.computeModel();
// //       ransac.getInliers(inliers);
// //       inliers_plane->indices=inliers;
// //       
// //       cout<<"aaaaaaaaaaaaaaaaaaaaaa"<<endl;
// //       
// //       extract.setInputCloud (cloud);
// //       extract.setIndices (inliers_plane);
// //       extract.setNegative (true);
// //       extract.filter(*final_);
//       
//       cout<<cloud->size()<<endl;
//       
//       pcl::PointCloud<PointT>::Ptr final2 (new pcl::PointCloud<PointT> (*cloud));
//       
//       if(rectprismFitting->isComputing())
//       {
//         world3D->update();
//       }
//       else
//       {
//         rectprismFitting->setCloud(final2);
//         rectprismFitting->start();
//         world3D->update();
//       }
// /*      
// 
//       if (!viewer.wasStopped())
//         viewer.showCloud (final_);*/
//       //Run fit cube
//     }
// 
//     void run ()
//     {
//       pcl::Grabber* interface = new pcl::OpenNIGrabber();
// 
//       boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
//         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
// 
//       interface->registerCallback (f);
// 
//       interface->start ();
// 
//       
//       
//       //interface->stop ();
//     }
//     
//     pcl::PointCloud<PointT>::Ptr final_;
//     std::vector<int> inliers;
//     pcl::PointIndices::Ptr inliers_plane; 
//     pcl::ExtractIndices<PointT> extract;
//     InnerModelManager *innerModelManager;
//     InnerModel *innerModel;
//     OsgView *world3D;
//     InnerModelViewer *imv;
//     RectPrismFitting *rectprismFitting;
// };

int main (int argc, char* argv[])
{
/*  
  QApplication app(argc, argv);
  SimpleOpenNIViewer v;
  v.run ();
  app.exec();*/
  
   QApplication app(argc, argv);
   
   myViewer m;
   m.cube();
   
   app.exec();
  
  return 0;
}
