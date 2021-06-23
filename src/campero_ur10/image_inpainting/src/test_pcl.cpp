#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h>

#include <tf/tf.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>


//#include <pcl_ros/surface/convex_hull.h> 
//#include <pcl/pcl_base.h>
//#include <pcl/point_types.h>
//#include <pcl/surface/concave_hull.h>

inline cv::Mat pclCloud2cvBin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  
  const int W = (int)cloud->width;
  const int H = (int)cloud->height;
  const int NUM_PTS = (int)cloud->points.size();
  ROS_INFO("N_pts: %d ", NUM_PTS);
  ROS_INFO("w: % d h: %d ", W, H);
  //ROS_INFO("Organized: %d ", _cloud.isOrganized());

  cv::Mat res = cv::Mat::zeros(cv::Size(W, H), CV_8UC1);
  ROS_INFO("Size: %d x %d ", res.rows, res.cols);
  for (int i = 0; i < NUM_PTS; i++)
  {
    pcl::PointXYZ pt = cloud->points[i];
    unsigned char& v = res.at<unsigned char>(pt.y, pt.x);
    v = 255;
  }

  return res;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr cvBin2pclCloud(const cv::Mat& img) {
    pcl::PointCloud<pcl::PointXYZ> _cloud;
    _cloud.width = img.cols; 
    _cloud.height = img.rows;
    _cloud.is_dense = false;
    _cloud.points.resize (_cloud.width * _cloud.height);
    ROS_INFO("Size: %d x %d ", img.rows, img.cols);
    for(int r = 0; r < img.rows; r++) {
        for(int c = 0; c < img.cols; c++) {
        const unsigned char& v = img.at<const unsigned char>(r, c);
        if (v == 255) {
            _cloud.at(c,r) = pcl::PointXYZ(c, r, 0);
        }
        }
    }
    ROS_INFO("N_pts: %d ", _cloud.size());
    ROS_INFO("w: % d h: %d ", _cloud.width, _cloud.height);
    ROS_INFO("Organized: %d ", _cloud.isOrganized());

    return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ> (_cloud));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pcl");
  ros::NodeHandle node_handle;

  std::string  img_file = argv[1];
  double alpha = std::atof(argv[2]);

  cv::Mat src = cv::imread(img_file);

  if (src.empty()) {
    ROS_ERROR("Imagen %s no es valida", img_file.c_str());
    ros::shutdown();
    return -1;
  }


  // 1.Binarizar
  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

  cv::Mat thresh;
  cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  
  // 2.Convert binary image -> pcl point
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cvBin2pclCloud(thresh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  // Build a filter to remove spurious NaNs and scene background
  ROS_INFO("Apply filter");
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  // proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->size () << " data points." << std::endl;

  
  //return 0;
  ROS_INFO("Convex hull with alpha %f ", alpha);
  pcl::ConcaveHull<pcl::PointXYZ> cHull;
  pcl::PointCloud<pcl::PointXYZ> cHull_points;
  cHull.setInputCloud(cloud_projected);
  cHull.setAlpha(alpha);
  cHull.setDimension(2);
  cHull.reconstruct(cHull_points);

  //cv::Mat res = pclCloud2cvBin(cloud);

  /*cv::imshow("orig", thresh);
  cv::imshow("cloud2img", res);
  cv::waitKey(0);
  cv::destroyAllWindows();*/
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud_projected);
  while (!viewer.wasStopped ())
  {
  }
  return 0;
}

int main2(int argc, char** argv)
{
  ros::init(argc, argv, "test_pcl");
  ros::NodeHandle node_handle;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  reader.read (argv[1], *cloud);
  std::cerr << "PointCloud init: "
            << cloud->size() << " data points." << std::endl;
  
  // Build a filter to remove spurious NaNs and scene background
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  // proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->size () << " data points." << std::endl;

  
  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->size ()
            << " data points." << std::endl;

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud_hull);
  while (!viewer.wasStopped ())
  {
  }
  //pcl::PCDWriter writer;
  //writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

  return (0);
}

