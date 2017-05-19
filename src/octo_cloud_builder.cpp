// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/PointStamped.h>
// #include <std_msgs/Float64.h>

// #include <pcl/common/transforms.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/point_types.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <tf2_ros/transform_listener.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// tf::TransformListener *tf_listener;

// void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan);

// std::string node_name = "cloud_builder";
// std::string topic_name_scan = "/scan_head";

// PointCloud::Ptr pclMapFrame;
// PointCloud::Ptr pclMapFrameFiltered;
// PointCloud::Ptr pcl2DMapFrame;
// PointCloud::Ptr pclCam;

// geometry_msgs::PoseWithCovarianceStamped current_pose;
// double current_tilt_angle;

// int main(int argc, char** argv)
// {
//   ros::init (argc, argv, node_name);
//   ros::NodeHandle nh;

//   ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>
//     (topic_name_scan, 10, laser_cb);

//   ros::Publisher pub_3D = nh.advertise<PointCloud> ("point_cloud_3D", 1);
//   ros::Publisher pub_2D = nh.advertise<PointCloud> ("point_cloud_2D", 1);

//   tf_listener = new tf::TransformListener();

//   PointCloud::Ptr tempCamCloud (new PointCloud);
//   pclCam = tempCamCloud;

//   PointCloud::Ptr tempFiltered (new PointCloud);
//   pclMapFrameFiltered = tempFiltered;

//   PointCloud::Ptr temp3D (new PointCloud);
//   pclMapFrame = temp3D;
  
//   PointCloud::Ptr temp2D (new PointCloud);
//   pcl2DMapFrame = temp2D;

//   ROS_INFO("[Cloud Builder] Initialized");

//   ros::Rate loop_rate(10);
//   while (nh.ok())
//   {
//     pclMapFrameFiltered->header.stamp = ros::Time::now().toNSec();
//     pclMapFrameFiltered->header.frame_id = "/map";
//     pub_3D.publish (pclMapFrameFiltered);

//     pcl2DMapFrame->header.stamp = ros::Time::now().toNSec();
//     pcl2DMapFrame->header.frame_id = "/map";
//     pub_2D.publish (pcl2DMapFrame);

//     ros::spinOnce ();
//     loop_rate.sleep ();
//   }
// }

// void laser_cb(const sensor_msgs::LaserScan::ConstPtr& _scan){
//   //Get the number of original laser reading
//   int originalCount = _scan->ranges.size();

//   double pan_angle_min = _scan->angle_min;
//   double pan_angle_max = _scan->angle_max;
//   double pan_angle_increment = _scan->angle_increment;

//   double pan_range_min = _scan->range_min;
//   double pan_range_max = _scan->range_max;

//   for (int i = 0; i < originalCount; i++){
//     int current_pan_index = i;
//     double current_pan_angle = pan_angle_min + current_pan_index * pan_angle_increment;

//     //ROS_INFO("Pan: %.3lf rad, %.3lf deg", current_pan_angle, current_pan_angle / 3.14 * 180);
//     //Laser frame
//     double theta = current_pan_angle;
//     double l = _scan->ranges[i];
//     double alpha = current_tilt_angle;

//     double temp_x = l * cos(theta);
//     double temp_y = l * sin(theta);
//     double temp_z = 0.0;

//     geometry_msgs::PointStamped temp_point_out;
//     geometry_msgs::PointStamped temp_point_in;
//     temp_point_in.header.frame_id = "/laser_head";
//     temp_point_in.point.x = temp_x;
//     temp_point_in.point.y = temp_y;
//     temp_point_in.point.z = temp_z;

//     try{
//         tf_listener->transformPoint("/map", temp_point_in, temp_point_out);
//     }
//     catch (tf::TransformException ex){
//         ROS_ERROR("%s",ex.what());
//         ros::Duration(1.0).sleep();
//     }
    
//     temp_x = temp_point_out.point.x;
//     temp_y = temp_point_out.point.y;
//     temp_z = temp_point_out.point.z;

//     pclMapFrame->points.push_back (pcl::PointXYZ(temp_x, temp_y, temp_z));
//   }

//   //Height filtering
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud (pclMapFrame);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0.2, 1.5);
//   //pass.setFilterLimitsNegative (true);
//   pass.filter (*pclMapFrameFiltered);

//   // Downsampling
//   pcl::VoxelGrid<pcl::PointXYZ> vox;
//   vox.setInputCloud (pclMapFrameFiltered);
//   vox.setLeafSize (0.02, 0.02, 0.02);
//   vox.filter (*pclMapFrameFiltered);

//   //Form the cam cloud
//   pcl::PointXYZ temp;
//   for (int j = 0; j < pclMapFrameFiltered->points.size(); j++){
//     temp = pclMapFrame->points.at(j);
//     double temp_x = temp.x;
//     double temp_y = temp.y;
//     double temp_z = temp.z;

//     double alpha = atan(temp_z / temp_x);
//     double temp_l = sqrt(temp_x * temp_x + temp_z * temp_z);
//     double theta = atan(temp_y / temp_l);
//     double temp_d = sqrt(temp_y * temp_y + temp_l * temp_l);

//     //ROS_INFO("Point: %.3lf, %.3lf, %.3lf", temp_x, temp_y, temp_z);

//     if (alpha < 0.1 && theta < 0.1 && temp_d < 2.0) {
//       pclCam->points.push_back (temp);
//       //ROS_INFO("FOUND");
//     }
//   }  

//   vox.setInputCloud (pclCam);
//   vox.setLeafSize (0.02, 0.02, 0.02);
//   vox.filter (*pclCam);

//   //2D projection
//   // Create a set of planar coefficients with X=Y=0,Z=1
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//   coefficients->values.resize (4);
//   coefficients->values[0] = coefficients->values[1] = 0;
//   coefficients->values[2] = 1.0;
//   coefficients->values[3] = 0;

//   // Create the filtering object
//   pcl::ProjectInliers<pcl::PointXYZ> proj;
//   proj.setModelType (pcl::SACMODEL_PLANE);
//   proj.setInputCloud (pclMapFrameFiltered);
//   proj.setModelCoefficients (coefficients);
//   proj.filter (*pcl2DMapFrame);
  
//   //Noise filtering
//   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//   sor.setInputCloud (pcl2DMapFrame);
//   sor.setMeanK (50);
//   sor.setStddevMulThresh (1.0);
//   sor.filter (*pcl2DMapFrame);
  
//   sor.setInputCloud (pclMapFrameFiltered);
//   sor.setMeanK (50);
//   sor.setStddevMulThresh (1.0);
//   sor.filter (*pclMapFrameFiltered);
// }


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"

using namespace std;
using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " spherical_scan.graph  (reference file to compare, required)\n\n";

  exit(1);
}

int main(int argc, char** argv) {
  if (argc != 2){
    printUsage(argv[0]);
  }

  std::string filename = std::string(argv[1]);
  
  ScanGraph referenceGraph;
  EXPECT_TRUE(referenceGraph.readBinary(filename));
  
  // TODO: read in reference graph file


  //##############################################################     

  point3d point_on_surface (4.01f, 0.01f, 0.01f);

  Pointcloud* cloud = new Pointcloud();

  for (int i=-50; i<51; i++) {
    for (int j=-50; j<51; j++) {
      point3d rotated = point_on_surface;
      rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
      cloud->push_back(rotated);
    }
  }
  
  pose6d origin(1.0, 0, -0.5, 0, 0, 0);
  
  ScanGraph graph;
  graph.addNode(cloud, origin); // graph assumes ownership of cloud!
  
  {
    std::cout << "Comparing ScanGraph with reference file at " << filename << std::endl;
    EXPECT_TRUE(graph.size() == referenceGraph.size());
    ScanNode* scanNode = *graph.begin();
    ScanNode* refScanNode = *referenceGraph.begin();
    
    EXPECT_EQ(scanNode->id, refScanNode->id);
    EXPECT_EQ(scanNode->pose, refScanNode->pose);
    EXPECT_EQ(scanNode->scan->size(), refScanNode->scan->size());

    for (size_t i = 0; i < scanNode->scan->size(); ++i){
      EXPECT_EQ((*scanNode->scan)[i], (*refScanNode->scan)[i]);
    }  
    
  }
  // test reading and writing to file
  {
    std::cout << "Testing ScanGraph I/O" << std::endl;
    
    EXPECT_TRUE(graph.writeBinary("spherical_scan_out.graph"));
    
    ScanGraph reReadGraph;
    EXPECT_TRUE(reReadGraph.readBinary("spherical_scan_out.graph"));
    
    EXPECT_TRUE(graph.size() == reReadGraph.size());
    EXPECT_EQ(reReadGraph.size(), 1);
    
    ScanNode* scanNode = *graph.begin();
    ScanNode* readScanNode = *reReadGraph.begin();
    
    EXPECT_EQ(scanNode->id, readScanNode->id);
    EXPECT_EQ(scanNode->pose, readScanNode->pose);
    EXPECT_EQ(scanNode->scan->size(), readScanNode->scan->size());

    for (size_t i = 0; i < scanNode->scan->size(); ++i){
      EXPECT_EQ((*scanNode->scan)[i], (*readScanNode->scan)[i]);
    }
  }
  
  
  // insert into OcTree  
  {
    OcTree tree (0.05);  

    // insert in global coordinates:
    tree.insertPointCloud(*cloud, origin.trans());

    tree.writeBinary("spherical_scan.bt");
  }
  
  cout << "Test done." << endl;
  exit(0);

}