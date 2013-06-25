#include "MCMCAdjust.h"


MCMCAdjust::MCMCAdjust(): r()
{
  this->weight=1;
}

void MCMCAdjust::estimateEigenAndCentroid(Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid)
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


void MCMCAdjust::initializeFromEigenValues()
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
  r.setCenter(QVec::vec3(centroid(0)-100, centroid(1)-100, centroid(2)-100));
  cout<<"Centroid: "<<centroid(0)<<" "<<centroid(1)<<" "<<centroid(2)<<endl;
  cout<<"Eigen Values/ratio: "<<(eigen_values(1)/ratio)*2<<" "<<(eigen_values(0)/ratio)*2 <<" "<<(eigen_values(2)/ratio)*2<<endl;
  
  
  //look at this!! wrong eigen_values loco! check this shit out
  
  //r.setWidth(QVec::vec3((eigen_values(0)/ratio),(eigen_values(0)/ratio),(eigen_values(0)/ratio)));
  r.setWidth(QVec::vec3(100,100,400));
  
  float rx = atan2(eigen_vectors(2,1), eigen_vectors(2,2));
  float ry = atan2(-eigen_vectors(2,0),sqrt(pow(eigen_vectors(2,1),2)+pow(eigen_vectors(2,2),2)));
  float rz = atan2(eigen_vectors(1,0),eigen_vectors(0,0));
  r.setRotation(QVec::vec3(0,0,0));
  
//   printf("max:    (%f, %f, %f)\n", max[0], max[1], max[2]);
//   printf("center: (%f, %f, %f)\n", center[0], center[1], center[2]);
//   printf("a:      (%f, %f, %f)\n", a[0], a[1], a[2]);
//   printf("b:      (%f, %f, %f)\n", b[0], b[1], b[2]);
//   printf("r:      %f\n\n", r);
  // Set data
//   c.setValues(a, b, r);

}

void MCMCAdjust::gypsyInitization()
{
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid (*data,centroid);
   
   r.setCenter(QVec::vec3(centroid(0), centroid(1), centroid(2)));
   r.setWidth(QVec::vec3(50,50,50));
   r.setRotation(QVec::vec3(0,0,0));

}

void MCMCAdjust::initialize()
{ 
   initializeFromEigenValues();
   //also to best
   best.setCenter(r.getCenter());
   best.setRotation(r.getRotation());
   best.setWidth(r.getWidth());

}

void MCMCAdjust::incTranslation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  QVec width = r.getWidth();
  float inc = width(index)/40;
  auxvec = r.getCenter();
  
  //decide direction
  auxvec(index)=auxvec(index)+inc;
  r.setCenter(auxvec);
  computeWeight();
  positiveWeight=weight;
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  r.setCenter(auxvec);
  computeWeight();
  negativeWeight=weight;
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  r.setCenter(auxvec);
  
  computeWeight();
  
    cout<<"positive: "<<positiveWeight<<" negative: "<<negativeWeight<<endl;
  //if negative is good go with it
  if(negativeWeight>positiveWeight)
  {
    transformedWeight=negativeWeight;
    inc*=-1;
  }
  else
    transformedWeight=positiveWeight;
    

  
  cout<<"Transformed: "<<transformedWeight<<" weight: "<<weight<<endl;
  while(transformedWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    r.setCenter(auxvec);
    weight=transformedWeight;
    
    //calculate futureWeight
    auxvec(index)=auxvec(index)+inc;
    r.setCenter(auxvec);
    computeWeight();
    transformedWeight=weight;
    
    //undo
    auxvec(index)=auxvec(index)-inc;
    r.setCenter(auxvec);
    computeWeight();
  }

//   QVec auxvec;
//   float positiveWeight, negativeWeight, transformedWeight;
//   QVec width = r.getWidth();
//   float inc = width(index)/40;
//   auxvec = r.getCenter();
//   
//   auxvec(index)=auxvec(index)+inc;
//   r.setCenter(auxvec);
//   computeWeight();
}

void MCMCAdjust::incRotation(int index)
{
  QVec auxvec;
  float positiveWeight, negativeWeight, transformedWeight;
  QVec width = r.getWidth();
  float inc = 0.02;
  auxvec = r.getRotation();
  
  //decide direction
  auxvec(index)=auxvec(index)+inc;
  r.setRotation(auxvec);
  computeWeight();
  positiveWeight=weight;
  
  //undo and sobstract a quarter = 2 quarters
  auxvec(index)=auxvec(index)-(inc*2);
  r.setRotation(auxvec);
  computeWeight();
  negativeWeight=weight;
  
  //undo changes
  auxvec(index)=auxvec(index)+inc;
  r.setRotation(auxvec);
  computeWeight();
  
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
    computeWeight();
    transformedWeight=weight;
    
    //undo
    auxvec(index)=auxvec(index)-inc;
    r.setRotation(auxvec);
    computeWeight();
  }
}

void MCMCAdjust::MarkovChainTranslation(int index)
{
  float inc = getRandom(RANDOM_SIZE);
  float currentWeight=weight;
  float nextWeight;
  QVec auxvec;
  
  auxvec=r.getCenter();
  
  //do and get weight
  auxvec(index)=auxvec(index)+inc;
  r.setCenter(auxvec);
  computeWeight();
  nextWeight=weight;
  //undo
  auxvec(index)=auxvec(index)-inc;
  r.setCenter(auxvec);
  computeWeight();
  
  //we get it for sure
  if(nextWeight>weight)
  {
    auxvec(index)=auxvec(index)+inc;
    r.setCenter(auxvec);
    computeWeight();
    //if better than best update best
    if(weight>bestweight)
    {
      best.setCenter(r.getCenter());
      best.setRotation(r.getRotation());
      best.setWidth(r.getWidth());
      bestweight=weight;
    }
  }
  //ifnot get it with probability nextweight/weight
  else
  {
    float probability=nextWeight/weight;
    if ( (((float) rand())/(float) RAND_MAX) < probability)
    {
      auxvec(index)=auxvec(index)+inc;
      r.setCenter(auxvec);
      computeWeight();
    }
  }
  
}

void MCMCAdjust::adapt ()
{
//   QVec auxvec;
  MarkovChainTranslation(0);
  MarkovChainTranslation(1);
  MarkovChainTranslation(2);
//   auxvec = r.getCenter();
  
//   incTranslation(0);
//     
//     
//     incTranslation(1);
//     incTranslation(2);
//     incRotation(0);
//     incRotation(1);
//     incRotation(2);
}

void MCMCAdjust::setData (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) 
{ 
  this->data = cloud; 
  computeWeight();
  bestweight=weight;
}


void MCMCAdjust::computeWeight()
{
 
  weight=0.;
  //estimate normals
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (data);
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (5);
  // Compute the features
  ne.compute (*cloud_normals);

  int normalint =0;
  for( pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = data->begin(); it != data->end(); it++ )
  {
    
    QVec point = QVec::vec3(it->x, it->y, it->z);
    QVec normal = QVec::vec3( cloud_normals->points[normalint].normal_x,cloud_normals->points[normalint].normal_y, cloud_normals->points[normalint].normal_z);


    double dist = r.distance(point,normal);
    weight += dist*dist;
    normalint++;
  }
  
  weight /= data->points.size();
  
  const float distance_weight = 1./(this->weight+0.1);

  
  weight=distance_weight;//*topbottom_weight;

}

QVec MCMCAdjust::getTranslation()
{
  return r.getCenter();
}

QVec MCMCAdjust::getRotation()
{
  return r.getRotation();
}

QVec MCMCAdjust::getScale()
{
  return r.getWidth();
}

void MCMCAdjust::print(std::string v)
{
  printf("%s: \n", v.c_str());
  printf("RectPrism: Center (%f,%f,%f), Rotation (%f,%f,%f), Width (%f,%f,%f), Weight: %f\n", 
	 r.getCenter()(0),r.getCenter()(1),r.getCenter()(2),
	 r.getRotation()(0),r.getRotation()(1),r.getRotation()(2),
	 r.getWidth()(0),r.getWidth()(1),r.getWidth()(2), weight);
}

float MCMCAdjust::getRandom(float var)
{
  double U = double(rand())/RAND_MAX;
  double V = double(rand())/RAND_MAX;
  return sqrt(-2*log(U))*cos(2.*M_PIl*V)*var;
}
