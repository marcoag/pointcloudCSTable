#ifndef RECTPRISM_H_
#define RECTPRISM_H_


#include <vector>
#include <bitset>
#include <math.h>
#include <cassert>
#include <QMat/QMatAll>
#include "axis.h"

#include <CGAL/Homogeneous.h>
#include <CGAL/squared_distance_3.h> 

typedef CGAL::Homogeneous<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Plane_3  Plane_3;
typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;


using namespace std;

class RectPrism
{
  
public:
  RectPrism();
  RectPrism(vector<double> center, vector<double> rotation, vector<double> width);
  inline vector<double> getCenter () { return center; }
  inline vector<double> getRotation () { return rotation; }
  inline vector<double> getWidth () { return width; }
  inline void setCenter ( vector<double> center ) { this->center=center; }
  inline void setRotation ( vector<double> rotation ) { this->rotation=rotation; }
  inline void setWidth ( vector<double> width ) { this->width=width; }
  bitset<6> cohenSutherland3D ( vector<double> point);
  
  double distance(vector<double> point);
  
private:
  vector<double> center;
  vector<double> rotation;
  vector<double> width;
  
};

#endif