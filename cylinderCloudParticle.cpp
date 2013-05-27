#include "cylinderCloudParticle.h"


CylinderCloudParticle::CylinderCloudParticle()
{
  Vector a(0,0,0);
  Vector b(0,0,1);
  c.setValues(a,b,1);
  this->weight=1;
//   varianceA=QVec::vec3(0, 0, 0);
//   varianceB=QVec::vec3(0, 0, 0);
//   varianceR=0;
  varianceA=QVec::vec3(10, 10, 10);
  varianceB=QVec::vec3(10, 10, 10);
  varianceR=10;
}

void CylinderCloudParticle::estimateEigenAndCentroid(const CylinderCloudPFInputData &data, Eigen::Vector3f &eig_values, Eigen::Matrix3f &eig_vectors, Eigen::Vector4f &centroid)
{
  static Eigen::Vector4f mean;
  static Eigen::Matrix3f cov;
  int np;
  
  centroid(0) = centroid(1) = centroid(2) = centroid(3) = 0.;
  np = 0;
  for (uint i=0; i<data.cloud_target->points.size(); i++)
  {
    if (isnan(data.cloud_target->points[i].z)) continue;
    centroid(0) += data.cloud_target->points[i].x;
    centroid(1) += data.cloud_target->points[i].y;
    centroid(2) += data.cloud_target->points[i].z;
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
  for (uint i=0; i<data.cloud_target->points.size(); i++)
  {
    tmp(0) = tmp(1) = tmp(2) = 0;
    if (isnan(data.cloud_target->points[i].z)) continue;
    tmp(0) = data.cloud_target->points[i].x - centroid(0);
    tmp(1) = data.cloud_target->points[i].y - centroid(1);
    tmp(2) = data.cloud_target->points[i].z - centroid(2);
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


void CylinderCloudParticle::initializeFromEigenValues(const CylinderCloudPFInputData &data){
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  
  pcl::compute3DCentroid (*data.cloud_target,centroid);
  computeCovarianceMatrix(*data.cloud_target,centroid,covariance_matrix);

 
  printf (" Size | %lu | \n", data.cloud_target->size());
  printf (" Centroid  | %f %f %f | \n", centroid (0), centroid (1), centroid (2));
  printf (" Covariance  | %f %f %f | \n", covariance_matrix (0,0), covariance_matrix (0,1), covariance_matrix (0,2));
  printf ("             | %f %f %f | \n", covariance_matrix (1,0), covariance_matrix (1,1), covariance_matrix (1,2));
  printf ("             | %f %f %f | \n", covariance_matrix (2,0), covariance_matrix (2,1), covariance_matrix (2,2));

  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
//   pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
  estimateEigenAndCentroid(data, eigen_values, eigen_vectors, centroid);

  printf (" EigenValues  | %f %f %f  | \n", eigen_values(0), eigen_values(1), eigen_values(2));
  printf (" EigenVectors | %f %f %f | \n", eigen_vectors (0,0), eigen_vectors (0,1), eigen_vectors (0,2));
  printf ("              | %f %f %f | \n", eigen_vectors (1,0), eigen_vectors (1,1), eigen_vectors (1,2));
  printf ("              | %f %f %f | \n", eigen_vectors (2,0), eigen_vectors (2,1), eigen_vectors (2,2));

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
  
  // Extract endpoints
  Vector max(eigen_vectors(max_position,0)*eigen_values(max_position),
             eigen_vectors(max_position,1)*eigen_values(max_position),
             eigen_vectors(max_position,2)*eigen_values(max_position));
  Vector center(centroid(0), centroid(1), centroid(2));

  Vector a = center + max;
  Vector b = center - max;
  double r = eigen_values(second_max_position)/2;
  

  
//   printf("max:    (%f, %f, %f)\n", max[0], max[1], max[2]);
//   printf("center: (%f, %f, %f)\n", center[0], center[1], center[2]);
//   printf("a:      (%f, %f, %f)\n", a[0], a[1], a[2]);
//   printf("b:      (%f, %f, %f)\n", b[0], b[1], b[2]);
//   printf("r:      %f\n\n", r);
  // Set data
  c.setValues(a, b, r);

}

void CylinderCloudParticle::gypsyInitization(const CylinderCloudPFInputData &data)
{
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid (*data.cloud_target,centroid);
   std::cout<<"Centroid: "<<centroid(0)<<" "<<centroid(1)<<" "<<centroid(2)<<std::endl;
   Vector center (centroid(0), centroid(1), centroid(2));
   Vector a (centroid(0), centroid(1) + 10, centroid(2));
   Vector b (centroid(0), centroid(1) - 10, centroid(2));
   double r=120;
   
   c.setValues(a,b,r);

}

void CylinderCloudParticle::initialize(const CylinderCloudPFInputData &data, const int &control, const RCParticleFilter_Config *config)
{ 
//   initializeFromEigenValues(data);
  gypsyInitization(data);
}

void CylinderCloudParticle::adapt(const int &controlBack, const int &controlNew, const bool noValidCandidates)
{
  Vector a(c.getA().getX()+getRandom(varianceA(0)), c.getA().getY()+getRandom(varianceA(1)), c.getA().getZ()+getRandom(varianceA(2)));
  Vector b(c.getB().getX()+getRandom(varianceB(0)), c.getB().getY()+getRandom(varianceB(1)), c.getB().getZ()+getRandom(varianceB(2)));
  double r = fabs(c.getR()+getRandom(varianceR));
  c.setValues(a,b,r);
  varianceA = varianceA.operator*(0.95);
  varianceB = varianceB.operator*(0.95);
  varianceR *= 0.95;
}

void CylinderCloudParticle::computeWeight(const CylinderCloudPFInputData &data)
{
//   printf("Cylinder: A(%f,%f,%f), B(%f,%f,%f), r=%f\n", c.getA().getX() , c.getA().getY() ,c.getA().getZ() , c.getB().getX() ,c.getB().getY() ,c.getB().getZ(), c.getR());
  this->weight=0.;
  double mint, maxt;
  for( pcl::PointCloud<pcl::PointXYZ>::iterator it = data.cloud_target->begin(); it != data.cloud_target->end(); it++ )
  {
    Vector point(it->x, it->y, it->z);
    double dist,t;
    const float distA = sqrt(c.R(point));   
    
    QVec qP = QVec::vec3(it->x, it->y, it->z);
    QVec qA = QVec::vec3(c.getA().getX(), c.getA().getY(), c.getA().getZ());
    QVec qB = QVec::vec3(c.getB().getX(), c.getB().getY(), c.getB().getZ());
    
    if (distA<0.0001)
    {
      const double distB = c.getR() - ((qP-qA).crossProduct(qP-qB)).norm2() / (qB-qA).norm2();
//       printf("metodo 2: %f\n", distB);
      dist = distB;
    }
    else
    {
//       printf("metodo 1: %f\n", distA);
      dist = distA;
    }
    this->weight += dist;
//     std::cout<<"X:"<<it->x<<" Y:"<<it->y<<" Z:"<<it->z<<" D:"<<dist<<std::endl;
    
    //look for the pcl and low points
    t=-((qA-qP).dotProduct(qB-qA))/fabs(pow((qB-qA).norm2(),2));
    if (t<mint || it==data.cloud_target->begin())
    {
      mint=t;
    }
    if (t>maxt || it==data.cloud_target->begin())
    {
      maxt=t;
    }
  }
  this->weight /= data.cloud_target->points.size();
  const float k = fabs(0.-mint)+fabs(1.-maxt);
  const float topbottom_weight = 1./(c.getR()*k+1);
  const float distance_weight = 1./(this->weight+1.);
//   std::cout<<"k: "<<k<<std::endl;
//   std::cout<<"topbottom_weight: "<<topbottom_weight<<" distance_weight: "<<distance_weight<<std::endl;
 // this->weight=distance_weight*topbottom_weight;
   this->weight=distance_weight;
//   printf("WEIGHT: %f\n", this->weight);
}

CylinderCloudParticle::~CylinderCloudParticle()
{

}

QVec CylinderCloudParticle::getTranslation() const
{
  return QVec::vec3(c.c[0], c.c[1], c.c[2]);
}

QVec CylinderCloudParticle::getRotation()
{
  QVec qA = QVec::vec3(c.getA().getX(), c.getA().getY(), c.getA().getZ());
  QVec qB = QVec::vec3(c.getB().getX(), c.getB().getY(), c.getB().getZ());

//   qA = QVec::vec3(0, -50, 0);
//   qB = QVec::vec3(0, 50, 0);
//   Rot3D r(0.5, 0.2, 0.2);
//   qA = r * qA;
//   qB = r * qB;
  
 
  QVec qAB = qA-qB;
  if (qA(1) < qB(1)) qAB = qAB.operator*(-1);
  
//   qAB.print("VECTOR");
  
  //Agustiiiiin!
  //float distIncl = -acos(qAB(2)/(qAB.norm2())) - M_PI_2;
  //float distAzim = atan2(qAB(0),(-1.0)*qAB(1));
  

//   float rx = asin(qAB(1)/sqrt(pow(qAB(1),2)+pow(qAB(2),2)));
//   float rz = asin(qAB(1)/sqrt(pow(qAB(0),2)+pow(qAB(1),2)));

  float rx = atan2(qAB(2), qAB(1));
  float rz = -atan2(qAB(0), qAB(1));
  
  return QVec::vec3(3.1415926535/2+rx,rz,0); // :-)
  
}

QVec CylinderCloudParticle::getScale() const
{
  return QVec::vec3(c.r[0], c.r[0], c.h);
}

float CylinderCloudParticle::getRandom(float var)
{
  double U = double(rand())/RAND_MAX;
  double V = double(rand())/RAND_MAX;
  return sqrt(-2*log(U))*cos(2.*M_PIl*V)*var;
}