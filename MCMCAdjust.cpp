#include "MCMCAdjust.h"


MCMCAdjust::MCMCAdjust(): r()
{
  this->weight=1;
  varianceC = QVec::vec3(5,5,5);
  varianceW = QVec::vec3(5,5,5);
  varianceR = QVec::vec3(0.1,0.1,0.1);
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
  r.setCenter(QVec::vec3(centroid(0)-20, centroid(1)-20, centroid(2)-20));
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

void MCMCAdjust::MarkovChainTranslationStepOnOne()
{
  //incs on each stuff
  int selection = rand()%3;
  QVec translationInc = QVec::vec3(getRandom(varianceC(0)),getRandom(varianceC(1)),getRandom(varianceC(2)));
  QVec rotationInc    = QVec::vec3(getRandom(varianceR(0)),getRandom(varianceR(1)),getRandom(varianceR(2)));
  QVec widthInc       = QVec::vec3(getRandom(varianceW(0)),getRandom(varianceW(1)),getRandom(varianceW(2)));
  
  float currentWeight=weight;
  float nextWeight;
  QVec translation = r.getCenter();
  QVec rotation = r.getRotation();
  QVec width = r.getWidth();
  
  //do change and get weight
  if(selection==0)
    r.setCenter(translation+translationInc);
  if(selection==1)
    r.setRotation(rotation+rotationInc);
  if(selection==2)
    r.setWidth(width+widthInc);  
  computeWeight();
  nextWeight=weight;
  
  //undo
  if(selection==0)
    r.setCenter(translation-translationInc);
  if(selection==1)
    r.setRotation(rotation-rotationInc);
  if(selection==2)
    r.setWidth(width-widthInc);   
  computeWeight();
  
  //we get it for sure
  if(nextWeight>weight)
  {
    if(selection==0)
      r.setCenter(translation+translationInc);
    if(selection==1)
      r.setRotation(rotation+rotationInc);
    if(selection==2)
      r.setWidth(width+widthInc); 
    computeWeight();
    //if better than best update best
    if(weight>bestweight)
    {
      if(selection==0)
        best.setCenter(r.getCenter());
      if(selection==1)
        best.setRotation(r.getRotation());
      if(selection==2)
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
      if(selection==0)
        r.setCenter(translation+translationInc);
      if(selection==1)
        r.setRotation(rotation+rotationInc);
      if(selection==2)
        r.setWidth(width+widthInc); 
      computeWeight();
    }
  }
}

void MCMCAdjust::MarkovChainTranslationStepOnAll()
{
  //incs on each stuff
  QVec translationInc = QVec::vec3(getRandom(varianceC(0)),getRandom(varianceC(1)),getRandom(varianceC(2)));
  QVec rotationInc    = QVec::vec3(getRandom(varianceR(0)),getRandom(varianceR(1)),getRandom(varianceR(2)));
  QVec widthInc       = QVec::vec3(getRandom(varianceW(0)),getRandom(varianceW(1)),getRandom(varianceW(2)));
  
  float currentWeight=weight;
  float nextWeight;
  QVec translation = r.getCenter();
  QVec rotation = r.getRotation();
  QVec width = r.getWidth();
  
  //do change and get weight
  r.setCenter(QVec::vec3(translation(0)+translationInc(0),translation(1)+translationInc(1),translation(2)+translationInc(2)));
  r.setRotation(QVec::vec3(rotation(0)+rotationInc(0),rotation(1)+rotationInc(1),rotation(2)+rotationInc(2)));
  r.setWidth(QVec::vec3(width(0)+widthInc(0),width(1)+widthInc(1),width(2)+widthInc(2)));  
  computeWeight();
  nextWeight=weight;
  
  //undo
  r.setCenter(QVec::vec3(translation(0)-translationInc(0),translation(1)-translationInc(1),translation(2)-translationInc(2)));
  r.setRotation(QVec::vec3(rotation(0)-rotationInc(0),rotation(1)-rotationInc(1),rotation(2)-rotationInc(2)));
  r.setWidth(QVec::vec3(width(0)-widthInc(0),width(1)-widthInc(1),width(2)-widthInc(2)));   
  computeWeight();
  
  //we get it for sure
  if(nextWeight>weight)
  {
    r.setCenter(QVec::vec3(translation(0)+translationInc(0),translation(1)+translationInc(1),translation(2)+translationInc(2)));
    r.setRotation(QVec::vec3(rotation(0)+rotationInc(0),rotation(1)+rotationInc(1),rotation(2)+rotationInc(2)));
    r.setWidth(QVec::vec3(width(0)+widthInc(0),width(1)+widthInc(1),width(2)+widthInc(2))); 
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
      r.setCenter(QVec::vec3(translation(0)+translationInc(0),translation(1)+translationInc(1),translation(2)+translationInc(2)));
      r.setRotation(QVec::vec3(rotation(0)+rotationInc(0),translation(1)+rotationInc(1),rotation(2)+rotationInc(2)));
      r.setWidth(QVec::vec3(width(0)+widthInc(0),width(1)+widthInc(1),width(2)+widthInc(2))); 
      computeWeight();
    }
  }
}

void MCMCAdjust::adapt ()
{
  MarkovChainTranslationStepOnOne();
//   varianceR.operator*(0.99);
//   varianceC.operator*(0.99);
//   varianceW.operator*(0.99);
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
    weight += dist;
    normalint++;
  }
  
   weight /= data->points.size();
//   
   const float distance_weight = 1./(this->weight+0.1);
// 
//   
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

QVec MCMCAdjust::getBestTranslation()
{
  return best.getCenter();
}

QVec MCMCAdjust::getBestRotation()
{
  return best.getRotation();
}

QVec MCMCAdjust::getBestScale()
{
  return best.getWidth();
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
