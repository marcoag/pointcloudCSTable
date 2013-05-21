#include "rectprism.h"

RectPrism::RectPrism()
{
  
  center.push_back(0.0);
  center.push_back(0.0);
  center.push_back(0.0);
  rotation.push_back(0.0);
  rotation.push_back(0.0);
  rotation.push_back(0.0);
  Wx=1.0;
  Wy=1.0;
  Wz=1.0;
  
}

RectPrism::RectPrism(vector<double> center, vector<double> rotation, double Wx, double Wy, double Wz)
{
  this->center=center;
  this->rotation=rotation;
  this->Wx=Wx;this->Wy=Wy;this->Wz=Wz;
}


void printCode(uint8_t code)
{
  for (uint8_t i=0; i<8; ++i)
  {
    uint8_t teta = (code<<(7-i));
    printf("%d", teta>>7 );
  }
  printf("\n");
}

double distance_p2p (double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
}

double RectPrism::distance(vector<double> point)
{
  
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
  float d1=(n1*t)(0,0)+(Wx/2);
  float d2=(n2*t)(0,0)+(Wy/2);
  float d3=(n3*t)(0,0)+(Wz/2);
  float d4=(n4*t)(0,0)-(Wx/2);
  float d5=(n5*t)(0,0)-(Wy/2);
  float d6=(n6*t)(0,0)-(Wz/2);

  n1.print("n1");
  n2.print("n2");
  n3.print("n3");
  n4.print("n4");
  n5.print("n5");
  n6.print("n6");
  
  std::cout<<d1<<std::endl;
  std::cout<<d2<<std::endl;
  std::cout<<d3<<std::endl;
  std::cout<<d4<<std::endl;
  std::cout<<d5<<std::endl;
  std::cout<<d6<<std::endl;
  
 
//  Object_handle o = cube.locate(Point_3(point[0],point[1],point[2]));
  
  
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


  uint8_t code=placePoint(point);
  
  printCode(code);
  
  switch(code)
  {
    case LAF: return distance_p2p(point[0],point[1],point[2],-(Wx/2),(Wy/2),-(Wz/2));
    case LAM: return sqrt(pow(point[0]-(-Wx/2),2)+pow(point[1]-(Wy/2),2));
    case LAB: return distance_p2p(point[0],point[1],point[2],-(Wx/2),(Wy/2),(Wz/2));
    case LMF: return sqrt(pow(point[0]-(-Wx/2),2)+pow(point[2]-(-Wz/2),2));
    case LMM: return point[0]-(-Wx/2);
    case LMB: return sqrt(pow(point[0]-(-Wx/2),2)+pow(point[2]-(Wz/2),2));
    case LBF: return distance_p2p(point[0],point[1],point[2],-(Wx/2),-(Wy/2),-(Wz/2));
    case LBM: return sqrt(pow(point[0]-(-Wx/2),2)+pow(point[1]-(-Wy/2),2));
    case LBB: return distance_p2p(point[0],point[1],point[2],-(Wx/2),-(Wy/2),(Wz/2));
    
    case MAF: return sqrt(pow(point[1]-(Wy/2),2)+pow(point[2]-(-Wz/2),2));
    case MAM: return point[1]-(Wy/2);
    case MAB: return sqrt(pow(point[1]-(Wy/2),2)+pow(point[2]-(Wz/2),2));
    case MMF: return point[2]-(-Wz/2);
    
    
    case MMM: 
      double distances[6];
      double min_distance;
      //-x
      distances[0]=abs((-Wx/2)-point[0]);
      //y
      distances[1]=abs((Wy/2)-point[1]);
      //-z
      distances[2]=abs((-Wz/2)-point[2]);
      //-y
      distances[3]=abs((-Wy/2)-point[1]);
      //z
      distances[4]=abs((Wz/2)-point[2]);
      //x
      distances[5]=abs((Wx/2)-point[0]);
      
      //get the minimum distance
      min_distance=distances[0];
      for(int i=1;i<6;i++)
      {
        if (min_distance>distances[i])
        {
          min_distance=distances[i];
        }
      }
      return min_distance;
    
    
    case MMB: return point[2]-(Wz/2);
    case MBF: return sqrt(pow(point[1]-(-Wy/2),2)+pow(point[2]-(-Wz/2),2));
    case MBM: return point[1]-(-Wy/2);
    case MBB: return sqrt(pow(point[1]-(-Wy/2),2)+pow(point[2]-(Wz/2),2));
    
    case RAF: return distance_p2p(point[0],point[1],point[2],(Wx/2),(Wy/2),-(Wz/2));
    case RAM: return sqrt(pow(point[0]-(Wx/2),2)+pow(point[1]-(Wy/2),2));
    case RAB: return distance_p2p(point[0],point[1],point[2],(Wx/2),(Wy/2),(Wz/2));
    case RMF: return sqrt(pow(point[0]-(Wx/2),2)+pow(point[2]-(-Wz/2),2));
    case RMM: printf("Point: %f, Wx/2: %f", point[0], Wx/2 );
      return point[0]-(Wx/2);
    case RMB: return sqrt(pow(point[0]-(Wx/2),2)+pow(point[2]-(Wz/2),2));
    case RBF: return distance_p2p(point[0],point[1],point[2],(Wx/2),-(Wy/2),(Wz/2));
    case RBM: return sqrt(pow(point[0]-(Wx/2),2)+pow(point[1]-(-Wy/2),2));
    case RBB: return distance_p2p(point[0],point[1],point[2],(Wx/2),-(Wy/2),-(Wz/2));
  }
  
  return -1;
}

uint8_t RectPrism::placePoint (vector<double> point)
{
  uint8_t code = 0;
  
  QVec qp = QVec::vec3(point[0],point[1],point[2]);
  QVec t = QVec::vec3(center[0],center[1],center[2]);
  //translate point back to cube
  qp=qp-t;
  
  //rotate around x
  qp(1)=qp(1)*cos(-rotation[0]) - qp(2)*sin(-rotation[0]); //y
  qp(2)=qp(2)*sin(-rotation[0]) - qp(1)*cos(-rotation[0]); //z
  //rotate around y
  qp(0)=qp(0)*cos(-rotation[1]) - qp(1)*sin(-rotation[1]); //x
  qp(2)=qp(2)*sin(-rotation[1]) - qp(0)*cos(-rotation[1]); //z
  //rotate around z
  qp(0)=qp(0)*cos(-rotation[2]) - qp(1)*sin(-rotation[2]); //x
  qp(1)=qp(0)*sin(-rotation[2]) - qp(1)*cos(-rotation[2]); //y
  
  //check X axis
  if (point[0]>(Wx/2))
    code=code||1<<XP;
  else if (point[0]<(-(Wx/2)))
    code=code||1<<XN;
  
  //check Y axis
  if (point[1]>(Wy/2))
    code=code||1<<YP;
  else if(point[1]<(-Wy/2))
    code=code||1<<YN;
  
  //check Z axis
  if (point[2]>(Wz/2))
    code=code||1<<ZP;
  else if(point[2]<((-Wz/2)))
    code=code||1<<ZN;
  
  return code;
}

