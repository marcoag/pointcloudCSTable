#ifndef RECTPRISM_H_
#define RECTPRISM_H_

#include <vector>
#include <math.h>
#include <QMat/QMatAll>
#include "axis.h"

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
  
  double distance(vector<double> point);
  
private:
  vector<double> center;
  vector<double> rotation;
  vector<double> width;
  
};

#endif