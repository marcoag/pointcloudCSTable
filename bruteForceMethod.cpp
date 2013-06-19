#include "bruteForceMethod.h"


BruteForceMethod::BruteForceMethod(): r()
{
  this->weight=1;
}

void BruteForceMethod::estimateEigenAndCentroid(Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid)
{
  static Eigen::Vector4f mean;
  static Eigen::Matrix3f cov;
  int np;
  
  centroid(0) = centroid(1) = centroid(2) = centroid(3) = 0.;
  np = 0;
  for (uint i=0; i<data->points.size(); i++)
  {
    if (isnan(data->points[i].z)) continue;
    centroid(0) += data->points[i].x;
    centroid(1) += data->points[i].y;
    centroid(2) += data->points[i].z;
    np += 1;
  }
  centroid(0) /= np;
  centroid(1) /= np;
  centroid(2) /= np;

  for (uint32_t ii=0 ; ii<3 ; ii++ )
    for (uint32_t jj=0 ; jj<3 ; jj++ )
      cov(ii,jj) = 0;

  Eigen::Vector4f tmp;
  tmp(3) = 0;
  for (uint i=0; i<data->points.size(); i++)
  {
    tmp(0) = tmp(1) = tmp(2) = 0;
    if (isnan(data->points[i].z)) continue;
    tmp(0) = data->points[i].x - centroid(0);
    tmp(1) = data->points[i].y - centroid(1);
    tmp(2) = data->points[i].z - centroid(2);
    for (uint32_t ii=0; ii<3; ii++)
      for (uint32_t jj=0; jj<3; jj++)
        cov(ii,jj) += tmp(ii)*tmp(jj);
  }
  cov /= np-1; 

  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  printf("x %f %f %f\n", centroid(0), centroid(1), centroid(2));
  // Choose the column vector with smallest eigenvalue
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver( cov );
  eig_values = solver.eigenvalues();
  eig_vectors = solver.eigenvectors();
}


void BruteForceMethod::initializeFromEigenValues()
{

  
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  
  pcl::compute3DCentroid (*data,centroid);
  computeCovarianceMatrix(*data,centroid,covariance_matrix);

 
//   printf (" Size | %lu | \n", data.cloud_target->size());
//   printf (" Centroid  | %f %f %f | \n", centroid (0), centroid (1), centroid (2));
//   printf (" Covariance  | %f %f %f | \n", covariance_matrix (0,0), covariance_matrix (0,1), covariance_matrix (0,2));
//   printf ("             | %f %f %f | \n", covariance_matrix (1,0), covariance_matrix (1,1), covariance_matrix (1,2));
//   printf ("             | %f %f %f | \n", covariance_matrix (2,0), covariance_matrix (2,1), covariance_matrix (2,2));

  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
//   pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
  
  estimateEigenAndCentroid(eigen_values, eigen_vectors, centroid);

//   printf (" EigenValues  | %f %f %f  | \n", eigen_values(0), eigen_values(1), eigen_values(2));
//   printf (" EigenVectors | %f %f %f | \n", eigen_vectors (0,0), eigen_vectors (0,1), eigen_vectors (0,2));
//   printf ("              | %f %f %f | \n", eigen_vectors (1,0), eigen_vectors (1,1), eigen_vectors (1,2));
//   printf ("              | %f %f %f | \n", eigen_vectors (2,0), eigen_vectors (2,1), eigen_vectors (2,2));

  // Extract max values and indices.
  float max_value = NAN;
  int max_position = -1; 
  float second_max_value = NAN;
  int second_max_position = -1;
  for (int i=0; i<3; i++)
  {
    if (eigen_values(i) > max_value || isnan(max_value) )
    {
      second_max_value=max_value;
      second_max_position=max_position;
      max_value=eigen_values(i);
      max_position=i;
    }
    else if (eigen_values(i)  > second_max_value || isnan(second_max_value))
    {
      second_max_value=eigen_values(i);
      second_max_position=i;
    }
  }
  
  //use the biggest eigen value
  float max_eigenvalue=0;
  if(max_eigenvalue - eigen_values(0) < 0)
    max_eigenvalue=eigen_values(0);
  if(max_eigenvalue - eigen_values(1) < 0)
    max_eigenvalue=eigen_values(1);
  if(max_eigenvalue - eigen_values(2) < 0)
    max_eigenvalue=eigen_values(2);
  
  float max_distance=0;
  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = data->points.begin (); it < data->points.end (); ++it)
  {
    //calculate point to point distance
//     float distance=fabs(RectPrism::distance_p2p(it->x, it->y, it->z, centroid(0), centroid(1), centroid(2)));
    float distance=fabs(sqrt(pow((it->x)-centroid(0),2.0)+pow((it->y)-centroid(2),2.0)+pow((it->z)-centroid(1),2.0)));
    if (max_distance<distance)
      max_distance=distance;
  }
//   cout<<"RectPrismCloudParticle::initializeFromEigenValues::max_distance: "<<max_distance<<endl;

  float ratio=max_eigenvalue/max_distance;
  cout<<"RectPrismCloudParticle::initializeFromEigenValues::Ratio: "<<ratio<<" max_distance:"<<max_distance<<endl;
  r.setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
  cout<<"Centroid: "<<centroid(0)<<" "<<centroid(1)<<" "<<centroid(2)<<endl;
  cout<<"Eigen Values/ratio: "<<(eigen_values(1)/ratio)*2<<" "<<(eigen_values(0)/ratio)*2 <<" "<<(eigen_values(2)/ratio)*2<<endl;
  
  
  //look at this!! wrong eigen_values loco! check this shit out
  
  //r.setWidth(QVec::vec3((eigen_values(0)/ratio),(eigen_values(0)/ratio),(eigen_values(0)/ratio)));
  r.setWidth(QVec::vec3(100,100,400));
  
  float rx = atan2(eigen_vectors(2,1), eigen_vectors(2,2));
  float ry = atan2(-eigen_vectors(2,0),sqrt(pow(eigen_vectors(2,1),2)+pow(eigen_vectors(2,2),2)));
  float rz = atan2(eigen_vectors(1,0),eigen_vectors(0,0));
  r.setRotation(QVec::vec3(0.1,0,0));
  
//   printf("max:    (%f, %f, %f)\n", max[0], max[1], max[2]);
//   printf("center: (%f, %f, %f)\n", center[0], center[1], center[2]);
//   printf("a:      (%f, %f, %f)\n", a[0], a[1], a[2]);
//   printf("b:      (%f, %f, %f)\n", b[0], b[1], b[2]);
//   printf("r:      %f\n\n", r);
  // Set data
//   c.setValues(a, b, r);

}

void BruteForceMethod::gypsyInitization()
{
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid (*data,centroid);
   Vector center (centroid(0), centroid(1), centroid(2));
   Vector a (centroid(0), centroid(1) + 50, centroid(2));
   Vector b (centroid(0), centroid(1) - 50, centroid(2));
   
   r.setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
   r.setWidth(QVec::vec3(50,50,50));
   r.setRotation(QVec::vec3(0,0,0));

}

void BruteForceMethod::initialize()
{ 
   initializeFromEigenValues();
   this->weight=computeWeight();
//  gypsyInitization(data);
}

void BruteForceMethod::incTranslation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  QVec width = r.getWidth();
  float inc = width(index)/40;
  auxvec = r.getCenter();
  

  //decide direction
  auxvec(index)=auxvec(index)+inc;
  r.setCenter(auxvec);
  
  positiveWeight=computeWeight();
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  r.setCenter(auxvec);
  
  negativeWeight=computeWeight();
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  r.setCenter(auxvec);
  
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  while(transformedWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    r.setCenter(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    r.setCenter(auxvec);
    transformedWeight=computeWeight();
    auxvec(index)=auxvec(index)-inc;
    r.setCenter(auxvec);
  }
}


void BruteForceMethod::incWidth(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  auxvec = r.getWidth();
  float inc = auxvec(index)/40;;
  

  //decide direction
  auxvec(index)=auxvec(index)+inc;
  r.setWidth(auxvec);
  
  positiveWeight=computeWeight();
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  r.setWidth(auxvec);
  
  negativeWeight=computeWeight();
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  r.setWidth(auxvec);
  
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  while(transformedWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    r.setWidth(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    r.setWidth(auxvec);
    transformedWeight=computeWeight();
    auxvec(index)=auxvec(index)-inc;
    r.setWidth(auxvec);
  }
}

void BruteForceMethod::incRotation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  auxvec = r.getRotation();
  float inc = 0.02;
  

  //decide direction
  auxvec(index)=auxvec(index)+inc;
  r.setRotation(auxvec);
  
  positiveWeight=computeWeight();
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  r.setRotation(auxvec);
  
  negativeWeight=computeWeight();
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  r.setRotation(auxvec);
  
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    
  while(transformedWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    r.setRotation(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    r.setRotation(auxvec);
    transformedWeight=computeWeight();
    auxvec(index)=auxvec(index)-inc;
    r.setRotation(auxvec);
  }
}

void BruteForceMethod::adapt ()
{

  switch(rand()%9)
  {
    //x
    case 0:
      incTranslation(0);
      break;
    //y
    case 1:
      incTranslation(1);
      break;
    //z
    case 2:
      incTranslation(2);
      break;
    //Wx
    case 3:
      incWidth(0);
      break;
    //Wy
    case 4:
      incWidth(1);
      break;  
    //Wz
    case 5:
      incWidth(2);
      break;
    //Rx
    case 6:
      incRotation(0);
      break;
    //Ry
    case 7:
      incRotation(1);
      break;
    //Rz
    case 8:
      incRotation(2);
      break;       
  }
}

void BruteForceMethod::setData (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) 
{ 
  this->data = cloud; 
  this->weight=computeWeight(); 
}

float BruteForceMethod::computeWeight()
{
//   printf("RectPrism: A(%f,%f,%f), B(%f,%f,%f), r=%f\n", c.getA().getX() , c.getA().getY() ,c.getA().getZ() , c.getB().getX() ,c.getB().getY() ,c.getB().getZ(), c.getR());
  float weight=0.;
  //double mint, maxt;
//  cout<<"TAMAÑO: "<<data.cloud_target->size()<<endl;
  
  //estimate normals
//   pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
//   ne.setInputCloud (data.cloud_target);
//   pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
//   ne.setSearchMethod (tree);
//   // Output datasets
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//   // Use all neighbors in a sphere of radius 3cm
//   ne.setRadiusSearch (5);
//   // Compute the features
//   ne.compute (*cloud_normals);

  
 // cout<<"Size cloud: "<<data.cloud_target->size()<<" size normals: "<<cloud_normals->size()<<endl;
  int normalint =0;
  for( pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = data->begin(); it != data->end(); it++ )
  {
    
    QVec point = QVec::vec3(it->x, it->y, it->z);
   // QVec normal = QVec::vec3( cloud_normals->points[normalint].normal_x,cloud_normals->points[normalint].normal_y, cloud_normals->points[normalint].normal_z);
    QVec normal = QVec::vec3(0,0,0);


//     double dist,t;
//     const float distA = sqrt(c.R(point));   
//     
//     QVec qP = QVec::vec3(it->x, it->y, it->z);
//     QVec qA = QVec::vec3(c.getA().getX(), c.getA().getY(), c.getA().getZ());
//     QVec qB = QVec::vec3(c.getB().getX(), c.getB().getY(), c.getB().getZ());
//     
//     if (distA<0.0001)
//     {
//       const double distB = c.getR() - ((qP-qA).crossProduct(qP-qB)).norm2() / (qB-qA).norm2();
// //       printf("metodo 2: %f\n", distB);
//       dist = distB;
//     }
//     else
//     {
// //       printf("metodo 1: %f\n", distA);
//       dist = distA;
//     }
    double dist = r.distance(point,normal);
    
    weight += dist;
//     std::cout<<"X:"<<it->x<<" Y:"<<it->y<<" Z:"<<it->z<<" D:"<<dist<<std::endl;
    
    //look for the upper and low points
//     t=-((qA-qP).dotProduct(qB-qA))/fabs(pow((qB-qA).norm2(),2));
//     if (t<mint || it==data.cloud_target->begin())
//     {
//       mint=t;
//     }
//     if (t>maxt || it==data.cloud_target->begin())
//     {
//       maxt=t;
//     }
    normalint++;
  }
  
  weight /= data->points.size();
  
  //const float k = fabs(0.-mint)+fabs(1.-maxt);
  //const float topbottom_weight = 1./(c.getR()*k+1);
  
  const float distance_weight = 1./(this->weight+0.1);
  
//   std::cout<<"k: "<<k<<std::endl;
//   std::cout<<"topbottom_weight: "<<topbottom_weight<<" distance_weight: "<<distance_weight<<std::endl;
  
  weight=distance_weight;//*topbottom_weight;
  
//   printf("WEIGHT: %f\n", this->weight);
  return weight;
}

BruteForceMethod::~BruteForceMethod()
{

}

QVec BruteForceMethod::getTranslation()
{
  return r.getCenter();
}

QVec BruteForceMethod::getRotation()
{
  return r.getRotation();
}

QVec BruteForceMethod::getScale()
{
  return r.getWidth();
}

void BruteForceMethod::setRectPrism (RectPrism r )
{
  this->r.setCenter ( r.getCenter() );
  this->r.setRotation ( r.getRotation() );
  this->r.setWidth ( r.getWidth() );
  
}

void BruteForceMethod::print(std::string v)
{
  printf("%s: \n", v.c_str());
  printf("RectPrism: Center (%f,%f,%f), Rotation (%f,%f,%f), Width (%f,%f,%f), Weight: %f\n", 
	 r.getCenter()(0),r.getCenter()(1),r.getCenter()(2),
	 r.getRotation()(0),r.getRotation()(1),r.getRotation()(2),
	 r.getWidth()(0),r.getWidth()(1),r.getWidth()(2), weight);
}
