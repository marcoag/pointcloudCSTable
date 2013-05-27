#include "rectprismFitting.h"

/**
  * \brief Default constructor
  */
RectPrismFitting::RectPrismFitting(InnerModelManager *imm): cloud_cup (new pcl::PointCloud<pcl::PointXYZ>),
QThread(),
computing(false)
{
  //sigset(SIGINT, sig_term); 
  innermodelManager = imm;
  
  c.particles=1000;
  
  //cup from kinect
   pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/cloud_cup.pcd", *cloud_cup);
  
  //Sintetic line
//   for(int i=1;i<1000;i++)
//   {
//     pcl::PointXYZ p;
//     p.x=i;
//     p.y=i;
//     p.z=0;
//     cloud_cup->push_back(p);
//   }
  
  //Sintetic cylinder
//   int h = 100;
//   int radio = 60;
//   Rot3D r(0.5, 0.2, 0.2);
//   for (float angle=0; angle<2.*M_PI;  angle+=0.05)//(M_PI_2-0.01))
//   {
//     for (float y = -h/2; y<h/2; y+=2)
//     {
//       pcl::PointXYZ p;
//       p.x = radio * cos(angle);
//       p.y = y;
//       p.z = radio * sin(angle);
//       QVec v = r * QVec::vec3(p.x, p.y, p.z);
//       p.x = v(0);
//       p.y = v(1);
//       p.z = v(2);
//       cloud_cup->push_back(p);
//     }
//   }
  

//   printf( "%s: %d\n", __FILE__, __LINE__);   
  innermodelManager->setPointCloudData("cup_cloud", cloud_cup);
  
  input.cloud_target=cloud_cup;  
  pf = new RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> (&c, input, 0);
  
  //Sintetic initialization
//   Vector aa(0,0,50);
//   Vector bb(0,0,-50);
//   double rad2= 1020;
//   Cylinder cylinder(aa,bb,rad2);
//   pf->weightedParticles[0].setCylinder(cylinder);
//   Vector a1(0,0,50);
//   Vector b1(0,0,-50);
//   double rad1= 60;
//   Cylinder cyl(a1,b1,rad1);
//   pf->weightedParticles[1].setCylinder(cyl);
}

/**
  * \brief Default destructor
  */
RectPrismFitting::~RectPrismFitting()
{

}

//Finish event
void RectPrismFitting::sig_term ()
{
  QApplication::exit();
}

// Vector RectPrismFitting::V(const double& r)
// {
//   Vector v(getRandom(), getRandom(), getRandom());
//     
//   v-=Vector(0.5);
//     
//   v*=2.0*r;
//   
//   return v;
//   
// }
void RectPrismFitting::run()
{ 
  computing=true;
//   Vector p(0,65,0);
//   
//   Vector a(0,50,0);
//   Vector b(0,-50,0);
// 
//   Cylinder cylinder(a,b,120);
// 
//   double d=sqrt(cylinder.R(p));
//   cout<<" distance: "<<d<<endl;
  
//   printf( "%s: %d\n", __FILE__, __LINE__);
// 
  pf->step(input, 0, false, -1);

  RectPrismCloudParticle bestParticle;

  
//   for (int i=0;i<c.particles; i++)
//   {
//     bestParticle = pf->weightedParticles[i];
//     std::cout<<i;
//     bestParticle.print(" resampled particle:");
//   }
  
  bestParticle = pf->getBest();
  //bestParticle = pf->getResampledParticle(0);
  bestParticle.print("bestParticle:");
  QVec t = bestParticle.getTranslation();
  QVec r = bestParticle.getRotation();
  QVec w = bestParticle.getScale();
//   t.print("T");
//   r.print("R");
//   s.print("S");

//   printf("%f %f %f\n", r(0), r(1), r(2));

  innermodelManager->setPose("cube_0_t", t, r, w );
  innermodelManager->setScale("cube_0", w(0), w(1), w(2));
//   catch(RoboCompInnerModelManager::InnerModelManagerError e)
//   {
//     std::cout<<e.text<<std::endl;
//   }
//   timer.stop();
//   getchar();
   computing=false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RectPrismFitting::readPCLCloud(QString name)
{
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  if (reader.read(name.toStdString(), *cloud) != -1)
    qFatal("No file %s", name.toStdString().c_str());
  return cloud;
}


