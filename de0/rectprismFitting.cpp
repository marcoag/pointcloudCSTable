#include "rectprismFitting.h"

/**
  * \brief Default constructor
  */
RectPrismFitting::RectPrismFitting(InnerModelManager *imm, boost::shared_ptr<QMutex> immMutex): QThread(),
computing(false),
cloudToFit(new pcl::PointCloud<PointT>())
{
  innermodelManager = imm;
  innerModelMutex=immMutex;
  makeSinteticCube();
  
  //Initialize method:
  bruteForceMethod = new BruteForceMethod();
  
  innerModelMutex->lock();
  innermodelManager->setPointCloudData("cup_cloud", cloudToFit);
  innerModelMutex->unlock();
  
  bruteForceMethod->setData(cloudToFit);
  bruteForceMethod->initialize();
  
}

void RectPrismFitting::run()
{ 
  computing=true;

  bruteForceMethod->adapt();
  cout<<""<<bruteForceMethod->getWeight()<<", ";
  
  QVec t = bruteForceMethod->getTranslation();
  QVec r = bruteForceMethod->getRotation();
  QVec w = bruteForceMethod->getScale();

  innerModelMutex->lock();
  innermodelManager->setPose("cube_0_t", t, r, w );
  innermodelManager->setScale("cube_0", w(0)/2, w(1)/2, w(2)/2);
  innerModelMutex->unlock();
  
  computing=false;
  
}


void RectPrismFitting::makeSinteticCube()
{

  //Sintetic cube
  int Wx = 100;
  int Wy = 100;
  int Wz = 400;
  int res = 20;
  //Rot3D r(0.5, 0.2, 0.2);
  //Faces front and back
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float y=0; y<=Wx; y=y+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = x+BruteForceMethod::getRandom(10);
      p.y = y+BruteForceMethod::getRandom(10);
      p.z = 0+BruteForceMethod::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloudToFit->push_back(p);
      p.x = x;
      p.y = y;
      p.z = Wz;  
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloudToFit->push_back(p);
    }
  }
  //Faces up and down
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = x+BruteForceMethod::getRandom(10);
      p.y = 0+BruteForceMethod::getRandom(10);
      p.z = z+BruteForceMethod::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloudToFit->push_back(p);
      p.x = x+BruteForceMethod::getRandom(10);
      p.y = Wy+BruteForceMethod::getRandom(10);;
      p.z = z+BruteForceMethod::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloudToFit->push_back(p);
    }
  }
  //Faces right and left
  for(float y=0; y<=Wy; y=y+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = 0+BruteForceMethod::getRandom(10);
      p.y = y+BruteForceMethod::getRandom(10);
      p.z = z+BruteForceMethod::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloudToFit->push_back(p);
      p.x = Wx+BruteForceMethod::getRandom(10);
      p.y = y+BruteForceMethod::getRandom(10);
      p.z = z+BruteForceMethod::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloudToFit->push_back(p);
    }
  }
  
//   float X = 0;
//   float Y = 0;
//   float Z = 100;
//   Eigen::Matrix4f TransMat; 
//   TransMat <<       1,    0,   0,  X, 
//                     0,    -0.4161,   -0.9093,  Y, 
//                     0,    0.9093,   -0.4161,  Z, 
//                     0,    0,   0,  1; 
//                     
//   pcl::transformPointCloud(*cloud_cup,*cloud_cup,TransMat ); 
  
}


