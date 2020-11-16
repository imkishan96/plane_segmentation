#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
Eigen::Vector3d u1, u2, u3 ,a , corrected_plane_normal , plane_normal;

tf::Transform transform;

tf::Quaternion q_tf;
tf::StampedTransform transform_in;

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

  double sgn = copysign(1.0, coefficients->values[2]); 
  
  corrected_plane_normal = {  coefficients->values[0] * sgn * -1.0,   // this is for the getting constant direction
                              coefficients->values[1] * sgn * -1.0,
                              coefficients->values[2] * sgn * -1.0};

//---------------this worked-------------testing ------100%  ----------------
  
  u1 = corrected_plane_normal.normalized();

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
  q = trans.getRotation();

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
//-------------------- Publishing TF---------------

  // transform.setOrigin( tf::Vector3( (double) plane_center.x(), 
  //                                   (double) plane_center.y(), 
  //                                   (double) plane_center.z() ) );
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));                                  
  transform.setRotation(q);
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "plane_normal_tf"));

// --------------------------- end -------------

  auto new_marker = marker;
  new_marker.id = 1;
  marker.ns = "my_namespace_1";
  new_marker.header.frame_id = "/odom";
  new_marker.pose.position.x = 0.0;
  new_marker.pose.position.y = 0.0;
  new_marker.pose.position.z = 0.0;
  tf::Quaternion q_new;
  q_new.setRPY(0.0,0.0,0.0);
  new_marker.pose.orientation.x = (double) (q_new * q).getX() ;
  new_marker.pose.orientation.y = (double) (q_new * q).getY() ;
  new_marker.pose.orientation.z = (double) (q_new * q).getZ() ;
  new_marker.pose.orientation.w = (double) (q_new * q).getW() ;
  new_marker.color.g = 0.0;
  new_marker.color.b = 1.0;
//---------------------------------------------

  pcl::toPCLPointCloud2(*cloud_pcl_f,*cloud_p);
  pub.publish(*cloud_p);
  pub_pn.publish(marker);
  pub_pn.publish(new_marker);
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
  pub_pn = nh.advertise<visualization_msgs::Marker>("plane_normal_marker", 1);
  // Spin

  // tf::TransformListener listener;

  // ros::Rate rate(100.0);
  // while (nh.ok()){

  //   try{
  //     listener.lookupTransform("odom", "camera_depth_optical_frame",
  //                              ros::Time(0), transform_in);
  //   }
  //   catch (tf::TransformException &ex) {
  //     ROS_ERROR("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //     continue;
  //   }
  //   rate.sleep();
  //   ROS_INFO("in the while loop");
  // } 



  
  ros::spin();
}