#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>


typedef pcl::PointXYZRGBA PointT;

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer")
    ,final_ (new pcl::PointCloud<PointT>)
    ,inliers_plane (new pcl::PointIndices)
    {
    }

    void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {

      pcl::SampleConsensusModelPlane<PointT>::Ptr
        model_s(new pcl::SampleConsensusModelPlane<PointT> (cloud));
      //Ransac
      pcl::RandomSampleConsensus<PointT> ransac (model_s);
      ransac.setDistanceThreshold (.01);
      ransac.computeModel();
      ransac.getInliers(inliers);
      inliers_plane->indices=inliers;
      
      extract.setInputCloud (cloud);
      extract.setIndices (inliers_plane);
      extract.setNegative (true);
      extract.filter(*final_);
      
      

      if (!viewer.wasStopped())
        viewer.showCloud (final_);
      //Run fit cube
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);

      interface->start ();

      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }

      interface->stop ();
    }
    
    pcl::PointCloud<PointT>::Ptr final_;
    std::vector<int> inliers;
    pcl::PointIndices::Ptr inliers_plane; 
    pcl::ExtractIndices<PointT> extract;
    pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}
