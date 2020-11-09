#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
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
#include <visualization_msgs/Marker.h>
#include <pcl/common/centroid.h>
#include <math.h> 
#include <Eigen/Geometry> 

ros::Publisher pub,pub_pn;
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

visualization_msgs::Marker marker;
Eigen::Vector4f plane_center;
//tf::Quaternion q;
tf::Vector3 plane_normal;
//tf::Vector3 up_vector(1.0, 0.0, 0.0);
//tf::Vector3 right_vector;
Eigen::Vector3d u1, u2, u3 ,a;


void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud)
{ 
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1.5, 1.5);
  pass.filter(*cloud_filtered);

  pass_1.setInputCloud(cloud_filtered_ptr);
  pass_1.setFilterFieldName("z");
  pass_1.setFilterLimits(0, 3);
  pass_1.filter(*cloud_filtered);

  // seg.segment(*inliers, *coefficients);
  sor.setInputCloud(cloud_filtered_ptr);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
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

  pcl::compute3DCentroid(*cloud_pcl_ptr, *inliers, plane_center);

  plane_normal = {coefficients->values[0], 
                  coefficients->values[1],
                  coefficients->values[2]};

//--------------------------------------testing ----------------------

  a = {coefficients->values[0], coefficients->values[1],
        coefficients->values[2]};

  u1 = a.normalized();

  if ((fabs((double)u1(0)) > 0.001) || (fabs((double)u1(1)) > 0.001)) {
      u2 << -u1(1), u1(0), 0;
  } else {
      u2 << 0, u1(2), -u1(1);
  }
  u2.normalize();

  u3 = u1.cross(u2);

  Eigen::Matrix3d R;  // Rotation matrix defining orientation
  R.col(0) = u1;
  R.col(1) = u2;
  R.col(2) = u3;  

  tf::Matrix3x3 tf_R;

  tf::matrixEigenToTF(R , tf_R);
  tf::Transform trans;
  trans.setBasis(tf_R);
  tf::Quaternion q;
  geometry_msgs::Quaternion gm_q;
  q = trans.getRotation();



//------------------------------ end -------------------------------
  marker.header.frame_id = "/camera_depth_optical_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = (double) plane_center.x();
  marker.pose.position.y = (double) plane_center.y();
  marker.pose.position.z = (double) plane_center.z();
  marker.pose.orientation.x = (double) q.getX() ;
  marker.pose.orientation.y = (double) q.getY() ;
  marker.pose.orientation.z = (double) q.getZ() ;
  marker.pose.orientation.w = (double) q.getW() ;
                 
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  ROS_INFO("%f  %f  %f  ", coefficients->values[0], 
        coefficients->values[1], coefficients->values[2]);
  pcl::toPCLPointCloud2(*cloud_pcl_f,*cloud_p);
  pub.publish(*cloud_p);
  pub_pn.publish(marker);
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
  pub_pn = nh.advertise<visualization_msgs::Marker>("plane_normal", 1);
  // Spin
  ros::spin();
}