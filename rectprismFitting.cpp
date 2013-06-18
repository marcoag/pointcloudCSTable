#include "rectprismFitting.h"

/**
  * \brief Default constructor
  */
RectPrismFitting::RectPrismFitting(InnerModelManager *imm): cloud_cup (new pcl::PointCloud<pcl::PointXYZRGBA>),
QThread(),
computing(false)
,final_(new pcl::PointCloud<PointT>())
,inliers_plane(new pcl::PointIndices ())
{
  innermodelManager = imm;
  
  c.particles=50;
  
  //cup from kinect
  //pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/cloud_cup.pcd", *cloud_cup);

#ifdef LIVE
  input.cloud_target = ransacAndEuclideanCluster(0.03f, 0.1f);  
#else
  input.cloud_target=getSinteticCube();
#endif

//   input.cloud_target=cloud;
  
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


pcl::PointCloud<PointT>::Ptr RectPrismFitting::getSinteticCube()
{
  pcl::PointCloud<PointT>::Ptr cloud_cup(new pcl::PointCloud<PointT>());
     //Sintetic cube
  int Wx = 100;
  int Wy = 100;
  int Wz = 400;
  int res = 5;
  //Rot3D r(0.5, 0.2, 0.2);
  //Faces front and back
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float y=0; y<=Wx; y=y+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = x;//+RectPrismCloudParticle::getRandom(10);
      p.y = y;//+RectPrismCloudParticle::getRandom(10);
      p.z = 0;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud_cup->push_back(p);
      p.x = x;
      p.y = y;
      p.z = Wz;  
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud_cup->push_back(p);
    }
  }
  //Faces up and down
  for(float x=0; x<=Wx; x=x+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = x;//+RectPrismCloudParticle::getRandom(10);
      p.y = 0;//+RectPrismCloudParticle::getRandom(10);
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud_cup->push_back(p);
      p.x = x;//+RectPrismCloudParticle::getRandom(10);
      p.y = Wy;//+RectPrismCloudParticle::getRandom(10);;
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud_cup->push_back(p);
    }
  }
  //Faces right and left
  for(float y=0; y<=Wy; y=y+res)
  {
    for(float z=0; z<=Wz; z=z+res)
    {
      //face front (x=0)
      pcl::PointXYZRGBA p;
      p.x = 0;//+RectPrismCloudParticle::getRandom(10);
      p.y = y;//+RectPrismCloudParticle::getRandom(10);
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud_cup->push_back(p);
      p.x = Wx;//+RectPrismCloudParticle::getRandom(10);
      p.y = y;//+RectPrismCloudParticle::getRandom(10);
      p.z = z;//+RectPrismCloudParticle::getRandom(10);
      p.r = 0;
      p.g = 255;
      p.b = 0;
      cloud_cup->push_back(p);
    }
  }
  
  float X = 0;
  float Y = 0;
  float Z = 0;
  Eigen::Matrix4f TransMat; 
  TransMat <<       1,    0,   0,  X, 
                    0,    -0.4161,   -0.9093,  Y, 
                    0,    0.9093,   -0.4161,  Z, 
                    0,    0,   0,  1; 
                    
  pcl::transformPointCloud(*cloud_cup,*cloud_cup,TransMat ); 
  
  return cloud_cup;
  
}

void RectPrismFitting::setCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    this->cloud=cloud;
}

/**
  * \brief Default constructor
  */
RectPrismFitting::RectPrismFitting(InnerModelManager *imm, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudToFit): QThread(),
computing(false)
,final_(new pcl::PointCloud<PointT>())
,inliers_plane(new pcl::PointIndices ())
{
  //sigset(SIGINT, sig_term); 
  innermodelManager = imm;
  
  c.particles=50;
  
  cloud = cloudToFit;
  
  cloud = ransacAndEuclideanCluster(0.03f, 0.1f);
  
  //lets try moving it to the center
//   Eigen::Vector4f centroid;
//   pcl::compute3DCentroid (*cloud,centroid);
//   
//   for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin (); it != cloud->end (); ++it)
//   {
//     it->x = it->x-centroid(0);
//     it->y = it->y-centroid(1);
//     it->z = it->z-centroid(2);
//   }
//   

#ifdef LIVE
  input.cloud_target=cloud;
  innermodelManager->setPointCloudData("cup_cloud", cloud);
#else
  input.cloud_target=getSinteticCube();
  innermodelManager->setPointCloudData("cup_cloud", getSinteticCube());
#endif
  
  
  pf = new RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> (&c, input, 0);
  
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

void RectPrismFitting::cloud2m(pcl::PointCloud<PointT>::Ptr cloud)
{
  for (uint i=0; i<cloud->points.size(); i++)
  {
    cloud->points[i].x *= 0.001;
    cloud->points[i].y *= 0.001;
    cloud->points[i].z *= 0.001;
  }
}

void RectPrismFitting::cloud2mm(pcl::PointCloud<PointT>::Ptr cloud)
{
  for (uint i=0; i<cloud->points.size(); i++)
  {
    cloud->points[i].x *= 1000;
    cloud->points[i].y *= 1000;
    cloud->points[i].z *= 1000;
  }
}


pcl::PointCloud<PointT>::Ptr RectPrismFitting::ransacAndEuclideanCluster(float ransacDistanceThreshold, float clusterTolerance)
{
    
    cloud2m(cloud);
  
    std::vector< int > nanindexes;
    pcl::removeNaNFromPointCloud(*cloud,*cloud,nanindexes);
  
    pcl::SampleConsensusModelPlane<PointT>::Ptr
      model_s(new pcl::SampleConsensusModelPlane<PointT> (cloud));
      
    //Ransac
    pcl::RandomSampleConsensus<PointT> ransac (model_s);
    ransac.setDistanceThreshold (0.02f);
    //ransac.setDistanceThreshold (5);
    ransac.computeModel();
    ransac.getInliers(inliers);
    inliers_plane->indices=inliers;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative (true);
    extract.filter(*final_);
    pcl::removeNaNFromPointCloud(*final_,*final_,nanindexes);
    
//     cout<<"MAAAAAAAAAAA: "<<final_->size()<<endl;
//     
//     for(int i=0;i<final_->size();i++)
//       cout<<final_->points[i]<<endl;
    
    
      //cluster extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (final_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.05f); // 2cm
    //ec.setClusterTolerance (10);
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (final_);
    ec.extract (cluster_indices);
    
    int j = 0;
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      if(j==0)
      {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (final_->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      }
      j++;
    }
    
    cloud2mm(cloud_cluster);
//    for(int i=0;i<cloud_cluster->size();i++)
//   {
//     cloud_cluster->points[i].x*=1000;
//     cloud_cluster->points[i].y*=1000;
//     cloud_cluster->points[i].z*=1000;
// 
//     cloud->points[i].x*=1000;
//     cloud->points[i].y*=1000;
//     cloud->points[i].z*=1000;
//   }
//   
  
  return cloud_cluster;
}

void RectPrismFitting::run()
{ 
  computing=true;

  //pcl::PointCloud<PointT>::Ptr cloud_cluster = ransacAndEuclideanCluster(0.03f, 0.1);

#ifdef LIVE  
  input.cloud_target=ransacAndEuclideanCluster(0.03f, 0.1f);
#else
  input.cloud_target=getSinteticCube();
#endif
    
  
  //innermodelManager->setPointCloudData("cup_cloud", input.cloud_target);
  
  pf->step(input, 0, false, -1);

  RectPrismCloudParticle bestParticle;

//   for (int i=0;i<c.particles; i++)
//   {
//     bestParticle = pf->weightedParticles[i];
//     std::cout<<i;
//     bestParticle.print(" resampled particle:");
//   }
  
  bestParticle = pf->getBest();
  bestParticle.print("bestParticle:");
  QVec t = bestParticle.getTranslation();
  QVec r = bestParticle.getRotation();
  QVec w = bestParticle.getScale();

  innermodelManager->setPose("cube_0_t", t, r, w );
  innermodelManager->setScale("cube_0", w(0)/2, w(1)/2, w(2)/2);

//   timer.stop();

   computing=false;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RectPrismFitting::readPCLCloud(QString name)
{
  
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  if (reader.read(name.toStdString(), *cloud) != -1)
    qFatal("No file %s", name.toStdString().c_str());
  return cloud;
  
}


