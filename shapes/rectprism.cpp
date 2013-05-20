#include "rectprism.h"

RectPrism::RectPrism()
{
  
  center.push_back(0.0);
  center.push_back(0.0);
  center.push_back(0.0);
  rotation.push_back(0.0);
  rotation.push_back(0.0);
  rotation.push_back(0.0);
  width.push_back(1.0);
  width.push_back(1.0);
  width.push_back(1.0);
  
}

RectPrism::RectPrism(vector<double> center, vector<double> rotation, vector<double> width)
{
  this->center=center;
  this->rotation=rotation;
  this->width=width;
}

double RectPrism::distance(vector<double> point)
{
  double distance = 0.0;
  
  QMat Rx(3,3);
  Rx(0,0)=1;               Rx(0,1)=0;               Rx(0,2)=0;
  Rx(1,0)=0;               Rx(1,1)=cos(rotation[0]);Rx(1,2)=-sin(rotation[0]);
  Rx(2,0)=0;               Rx(2,1)=sin(rotation[0]);Rx(2,2)=cos(rotation[0]);
  
  QMat Ry(3,3);
  Ry(0,0)=cos(rotation[1]); Ry(0,1)=0;              Ry(0,2)=sin(rotation[1]);
  Ry(1,0)=0;                Ry(1,1)=1;              Ry(1,2)=0;
  Ry(2,0)=-sin(rotation[1]);Ry(2,1)=0;              Ry(2,2)=cos(rotation[1]);
  
  QMat Rz(3,3);
  Rz(0,0)=cos(rotation[2]); Rz(0,1)=-sin(rotation[1]); Rz(0,2)=0;
  Rz(1,0)=sin(rotation[1]); Rz(1,1)=cos(rotation[1]);  Rz(1,2)=0;
  Rz(2,0)=0;                Rz(2,1)=0;                 Rz(2,2)=1;
  
  //rotate around x
//   rotatedCenter[1]=center[1]*cos(Rx) - center[2]*sin(Rx); //y
//   rotatedCenter[2]=center[2]*sin(Rx) - center[1]*cos(Rx); //z
//   //rotate around y
//   rotatedCenter[0]=center[0]*cos(Ry) - center[1]*sin(Ry); //x
//   rotatedCenter[2]=center[2]*sin(Ry) - center[0]*cos(Ry); //z
//   //rotate around z
//   rotatedCenter[0]=center[0]*cos(Rz) - center[1]*sin(Rz); //x
//   rotatedCenter[1]=center[0]*sin(Rz) - center[1]*cos(Rz); //y
//   

  //Plane normals
  QVec n1=QVec::vec3(0,0,1);
  QVec n2=QVec::vec3(0,1,0);
  QVec n3=QVec::vec3(1,0,0);
  QVec n4=QVec::vec3(0,0,-1);
  QVec n5=QVec::vec3(0,-1,0);
  QVec n6=QVec::vec3(-1,0,0);
  
  //rotate planes
  n1=n1*Rx;
  n1=n1*Ry;
  n2=n2*Rx;
  n2=n2*Rz;
  n3=n3*Ry;
  n3=n3*Rz;
  n4=n4*Rx;
  n4=n4*Ry;
  n5=n5*Rx;
  n5=n5*Rz;
  n6=n6*Ry;
  n6=n6*Rz;

  //Translate vector
  QMat t(3,1);
  t(0,0)=center[0];
  t(1,0)=center[1];
  t(2,0)=center[2];
  
  //calculate distances
  float d1=(n1*t)(0,0)+(width[0]/2);
  float d2=(n2*t)(0,0)+(width[1]/2);
  float d3=(n3*t)(0,0)+(width[2]/2);
  float d4=(n4*t)(0,0)-(width[0]/2);
  float d5=(n5*t)(0,0)-(width[1]/2);
  float d6=(n6*t)(0,0)-(width[2]/2);

  n1.print("n1");
  n2.print("n2");
  n3.print("n3");
  n4.print("n4");
  n5.print("n5");
  n6.print("n6");
  
  std::cout<<d1mat<<std::endl;
  std::cout<<d2mat<<std::endl;
  std::cout<<d3mat<<std::endl;
  std::cout<<d4mat<<std::endl;
  std::cout<<d5mat<<std::endl;
  std::cout<<d6mat<<std::endl;
  
  Plane_3 p1(n1(0),n1(1),n1(2),d1);
  Plane_3 p2(n2(0),n2(1),n2(2),d2);
  Plane_3 p3(n3(0),n3(1),n3(2),d3);
  Plane_3 p4(n4(0),n4(1),n4(2),d4);
  Plane_3 p5(n5(0),n5(1),n5(2),d5);
  Plane_3 p6(n6(0),n6(1),n6(2),d6);
  
  Polyhedron P;
  
  //object_handle o = cube.ray_shoot_to_boundary (Point(point[0],point[1],point[2]));
  
  
//   // the cube [0,1]^3
//   Point P[8] = { Point(0,0,0), Point(0,0,1), Point(0,1,0), Point(0,1,1),
//                  Point(1,0,0), Point(1,0,1), Point(1,1,0), Point(1,1,1)};
// 
//   // the cube [2,3]^3
//   Point Q[1] = { Point(point[0], point[1], point[2]) };
//   
//   Polytope_distance pd(P, P+8, Q, Q+1);
//   assert (pd.is_valid());
//   
//   distance = sqrt(CGAL::to_double (pd.squared_distance_numerator()) /
//     CGAL::to_double (pd.squared_distance_denominator()));
  
  
  if (distance == 0)
    
  
  return distance;
}

bitset<6> RectPrism::cohenSutherland3D (vector<double> point)
{
    bitset<6> code(string("000000"));
    
    //calculate line:

    
    return code;
}

