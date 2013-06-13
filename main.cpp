#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <QWidget>

#include <osgviewer/osgview.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include "myViewer.h"
#include "innermodelManager.h"
#include "rectprismFitting.h"



int main (int argc, char* argv[])
{
   QApplication app(argc, argv);
   
   myViewer m;
   m.cube();
   app.exec();
  
  return 0;
}
