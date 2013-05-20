#ifndef RECTPRISM_H_
#define RECTPRISM_H_

#include <vector>
#include <bitset>
#include <math.h>
#include <cassert>
#include <QMat/QMatAll>
#include "axis.h"
#include "codes.h"

#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/squared_distance_3.h> 



//typedef CGAL::Homogeneous<double> Kernel;
// typedef CGAL::Extended_homogeneous<CGAL::Gmpz>  Kernel;
// typedef Kernel::Point_3 Point_3;
// typedef Kernel::Plane_3  Plane_3;
// typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;
// typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron_3;
// typedef Nef_polyhedron_3::Object_handle Object_handle;
// typedef Nef_polyhedron_3::Vertex_const_iterator  Vertex_const_iterator;

using namespace std;

class RectPrism
{
  
public:
  RectPrism();
  RectPrism(vector<double> center, vector<double> rotation, double Wx, double Wy, double Wz);
  inline vector<double> getCenter () { return center; }
  inline vector<double> getRotation () { return rotation; }
  inline void setCenter ( vector<double> center ) { this->center=center; }
  inline void setRotation ( vector<double> rotation ) { this->rotation=rotation; }
  inline void setWidth ( double Wx, double Wy, double Wz ) { this->Wx=Wx;this->Wy=Wy;this->Wz=Wz; }
  uint8_t placePoint ( vector<double> point);
  
  double distance(vector<double> point);
  
private:
  vector<double> center;
  vector<double> rotation;
  double Wx, Wy, Wz;
  
};

#endif