#include <ros/ros.h>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>

ros::Publisher pub;
pcl::PassThrough<pcl::PCLPointCloud2> pass, pass_1;
pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2 *cloud_p = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloud_filtered_ptr(cloud_filtered);
pcl::PCLPointCloud2ConstPtr cloud_p_ptr(cloud_p);

pcl::PointCloud<pcl::PointXYZ> *cloud_pcl = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ> *cloud_pcl_f = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_ptr(cloud_pcl);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_f_ptr(cloud_pcl_f);

pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ExtractIndices<pcl::PointXYZ> extract;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud)
{ 
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.5, 0.5);
  pass.filter(*cloud_filtered);

  pass_1.setInputCloud(cloud_filtered_ptr);
  pass_1.setFilterFieldName("z");
  pass_1.setFilterLimits(0, 2);
  pass_1.filter(*cloud_filtered);

  // seg.segment(*inliers, *coefficients);
  sor.setInputCloud(cloud_filtered_ptr);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_filtered);

  pcl::fromPCLPointCloud2( *cloud_filtered, *cloud_pcl_ptr);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);
  seg.setInputCloud(cloud_pcl_ptr);
  seg.segment(*inliers, *coefficients);

  extract.setInputCloud(cloud_pcl_ptr);
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.filter(*cloud_pcl_f_ptr);

  pcl::toPCLPointCloud2(*cloud_pcl_f,*cloud_p);
  pub.publish(*cloud_p);
  //pub.publish(*cloud_filtered);
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "rs_pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2>("output", 1);

  // Spin
  ros::spin();
}