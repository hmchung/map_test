#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
tf::TransformListener *tf_listener;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan);
void scan_head_status_cb(const std_msgs::Int8::ConstPtr& _flag);

bool reset_cloud_flag = false;
int signal_value = 1;
int signal_count = 0;
int signal_trigger_count = 2;

std::string node_name = "cloud_builder";
std::string topic_name_scan = "/scan_head";
std::string topic_name_head_status = "/ugv_1/laser_scan_state";

PointCloud::Ptr pclMapFrame;
PointCloud::Ptr pclMapFrameFiltered;
PointCloud::Ptr pclMapFrameFiltered_LastFull;
PointCloud::Ptr pcl2DMapFrame;
PointCloud::Ptr pclRawLaser;
PointCloud::Ptr pclCam;

geometry_msgs::PoseWithCovarianceStamped current_pose;
double current_tilt_angle;

ros::Publisher pub_raw_laser;

int main(int argc, char** argv)
{
  ros::init (argc, argv, node_name);
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/octomap_server/reset");

  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>
    (topic_name_scan, 10, laser_cb);
  ros::Subscriber scan_head_status_sub = nh.subscribe<std_msgs::Int8>
    (topic_name_head_status, 10, scan_head_status_cb);

  ros::Publisher pub_3D = nh.advertise<PointCloud> ("point_cloud_3D", 1);
  ros::Publisher pub_3D_last = nh.advertise<PointCloud> ("point_cloud_3D_last", 1);
  // ros::Publisher pub_2D = nh.advertise<PointCloud> ("point_cloud_2D", 1);
  pub_raw_laser = nh.advertise<PointCloud> ("point_cloud_raw_laser", 1);

  tf_listener = new tf::TransformListener();

  PointCloud::Ptr tempRawLaser (new PointCloud);
  pclRawLaser = tempRawLaser;

  PointCloud::Ptr tempCamCloud (new PointCloud);
  pclCam = tempCamCloud;

  PointCloud::Ptr tempFiltered (new PointCloud);
  pclMapFrameFiltered = tempFiltered;
  pclMapFrameFiltered_LastFull = tempFiltered;

  PointCloud::Ptr temp3D (new PointCloud);
  pclMapFrame = temp3D;
  
  PointCloud::Ptr temp2D (new PointCloud);
  pcl2DMapFrame = temp2D;

  ROS_INFO("[Cloud Builder] Initialized");

  ros::Rate loop_rate(10);
  while (nh.ok())
  {
    try{
      //pclMapFrameFiltered->header.stamp = ros::Time::now().toNSec();
      // if (reset_cloud_flag){
      //   reset_cloud_flag = false;
      // }
      pclMapFrameFiltered->header.frame_id = "/map";
      pclMapFrameFiltered_LastFull->header.frame_id = "/map"; 
      pub_3D.publish (pclMapFrameFiltered);
      pub_3D_last.publish (pclMapFrameFiltered_LastFull);
    }catch(std::runtime_error& ex) {
      ROS_ERROR("Exception: [%s]", ex.what());
    }
    // pcl2DMapFrame->header.stamp = ros::Time::now().toNSec();
    // pcl2DMapFrame->header.frame_id = "/map";
    // pub_2D.publish (pcl2DMapFrame);

    ros::spinOnce ();
    loop_rate.sleep ();
  }
}

void scan_head_status_cb(const std_msgs::Int8::ConstPtr& _flag){
  int temp_flag = _flag->data;
  if (temp_flag == signal_value){
    signal_count++;
    if ((signal_count % signal_trigger_count == 0)){
        pclMapFrameFiltered_LastFull = pclMapFrame;
        PointCloud::Ptr blank_cloud (new PointCloud);
        pclMapFrame = blank_cloud;
    }
  }
}


void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan){
  //Get the number of original laser reading
  int originalCount = _scan->ranges.size();

  double pan_angle_min = _scan->angle_min;
  double pan_angle_max = _scan->angle_max;
  double pan_angle_increment = _scan->angle_increment;

  double pan_range_min = _scan->range_min;
  double pan_range_max = _scan->range_max;

  PointCloud::Ptr tempRawLaser2 (new PointCloud);

  for (int i = 0; i < originalCount; i++){
    int current_pan_index = i;
    double current_pan_angle = pan_angle_min + current_pan_index * pan_angle_increment;

    //ROS_INFO("Pan: %.3lf rad, %.3lf deg", current_pan_angle, current_pan_angle / 3.14 * 180);
    //Laser frame
    double theta = current_pan_angle;
    double l = _scan->ranges[i];
    double alpha = current_tilt_angle;

    double temp_x = l * cos(theta);
    double temp_y = l * sin(theta);
    double temp_z = 0.0;

    geometry_msgs::PointStamped temp_point_out;
    geometry_msgs::PointStamped temp_point_in;
    temp_point_in.header.frame_id = "/laser_head";
    temp_point_in.point.x = temp_x;
    temp_point_in.point.y = temp_y;
    temp_point_in.point.z = temp_z;

    try{
        tf_listener->transformPoint("/map", temp_point_in, temp_point_out);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    temp_x = temp_point_out.point.x;
    temp_y = temp_point_out.point.y;
    temp_z = temp_point_out.point.z;

    //tempRawLaser2->points.push_back(pcl::PointXYZ(temp_x, temp_y, temp_z));
    pclMapFrame->points.push_back (pcl::PointXYZ(temp_x, temp_y, temp_z));
  }

  //pclRawLaser = tempRawLaser2;
  //pclRawLaser->header.stamp = ros::Time::now().toNSec();
  //pclRawLaser->header.frame_id = "/map";
  //pub_raw_laser.publish (pclRawLaser);

  // Height filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pclMapFrame);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 3.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*pclMapFrameFiltered);

  // Downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (pclMapFrameFiltered);
  vox.setLeafSize (0.01, 0.01, 0.01);
  vox.filter (*pclMapFrameFiltered);

  //////////Form the cam cloud
  // pcl::PointXYZ temp;
  // for (int j = 0; j < pclMapFrameFiltered->points.size(); j++){
  //   temp = pclMapFrame->points.at(j);
  //   double temp_x = temp.x;
  //   double temp_y = temp.y;
  //   double temp_z = temp.z;

  //   double alpha = atan(temp_z / temp_x);
  //   double temp_l = sqrt(temp_x * temp_x + temp_z * temp_z);
  //   double theta = atan(temp_y / temp_l);
  //   double temp_d = sqrt(temp_y * temp_y + temp_l * temp_l);
  //   if (alpha < 0.1 && theta < 0.1 && temp_d < 2.0) {
  //     pclCam->points.push_back (temp);
  //   }
  // }  

  // vox.setInputCloud (pclCam);
  // vox.setLeafSize (0.02, 0.02, 0.02);
  // vox.filter (*pclCam);

  /////////////2D projection
  // Create a set of planar coefficients with X=Y=0,Z=1
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  // coefficients->values.resize (4);
  // coefficients->values[0] = coefficients->values[1] = 0;
  // coefficients->values[2] = 1.0;
  // coefficients->values[3] = 0;

  // Create the filtering object
  // pcl::ProjectInliers<pcl::PointXYZ> proj;
  // proj.setModelType (pcl::SACMODEL_PLANE);
  // proj.setInputCloud (pclMapFrameFiltered);
  // proj.setModelCoefficients (coefficients);
  // proj.filter (*pcl2DMapFrame);
  
  //Noise filtering
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (pcl2DMapFrame);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*pcl2DMapFrame);
  
  // sor.setInputCloud (pclMapFrameFiltered);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*pclMapFrameFiltered);
}