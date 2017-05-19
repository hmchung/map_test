#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

std::string node_name = "map_builder_2";

std::string pub_name_map = "/map";

//map properties
const int map_width = 2000; // cells
const int map_height = 2000; // cells
float map_resolution = 0.05; // m/cell
int map_data[map_width][map_height];

ros::Publisher pub;
sensor_msgs::PointCloud2 current_2D_cloud;

bool has_cloud = false;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr pcl2DMapFrame;

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	//Subscriber
	ros::Subscriber sub_2d_cloud = nh.subscribe("/point_cloud_3D", 1, cloud_cb);
	//Publisher
	ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>(pub_name_map, 10);
	
	//Map preparation


	ros::Rate loop_rate(30);

	while (ros::ok()){
		if (has_cloud){
			nav_msgs::OccupancyGrid my_map;
			my_map.header.stamp = ros::Time::now();
			my_map.header.frame_id = "map";

			my_map.info.resolution = map_resolution;
			my_map.info.width = map_width;
			my_map.info.height = map_height;
			my_map.info.origin.position.x = - (map_height * map_resolution) / 2;
			my_map.info.origin.position.y = - (map_width * map_resolution) / 2;
			my_map.info.origin.position.z = 0;		


			//Prepare empty map
			for (int j = 0; j < map_height; j++){
				for (int i = 0; i < map_width; i++){
					map_data[i][j] = 0;
				}
			}

	  		pcl::PointXYZ temp;
			for (int k = 0; k < pcl2DMapFrame->points.size(); k++){
				temp = pcl2DMapFrame->points.at(k);
				double temp_x = temp.x;
				double temp_y = temp.y;

				int temp_i = temp_x / map_resolution + map_height / 2;
				int temp_j = temp_y / map_resolution + map_width / 2;

				if (temp_i >= 0 && temp_j >= 0){
					map_data[temp_i][temp_j] = 100;
				}
				
			}  

			// //Draw objects
			// //for (int obj_index = 0; obj_index < object_queue.size(); obj_index++){
			// while(!object_queue.empty()){
			// 	Small_Object temp_object = object_queue.front();
			// 	object_queue.pop();
			// 	object_queue_swap.push(temp_object);
			// 	//dummy object
			// 	float object_x = temp_object.my_position.x;
			// 	float object_y = temp_object.my_position.y;
			// 	float object_d = temp_object.my_size;

			// 	for (int j = 0; j < map_height; j++){
			// 		for (int i = 0; i < map_width; i++){
			// 			float temp_dx = i * map_resolution - object_x;
			// 			float temp_dy = j * map_resolution - object_y;
			// 			float temp_distance = sqrt(temp_dx * temp_dx + temp_dy * temp_dy);
			// 			if (temp_distance < object_d)  map_data[i][j] = 100;
			// 		}
			// 	}
			// 	//ROS_INFO("Object: %d", obj_index);
			// }

			//Put data into map
			for (int j = 0; j < map_height; j++){
				for (int i = 0; i < map_width; i++){
					my_map.data.push_back(map_data[i][j]);
				}
			}

			pub_map.publish(my_map);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	current_2D_cloud = *input;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
	
	//2D projection
	//Height filtering
	// pcl::PassThrough<pcl::PointXYZ> pass;
	// pass.setInputCloud (temp_cloud);
	// pass.setFilterFieldName ("z");
	// pass.setFilterLimits (0.2, 1.5);
	// //pass.setFilterLimitsNegative (true);
	// pass.filter (*temp_cloud);

	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (temp_cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*temp_cloud);

	//Noise filtering
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (temp_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*temp_cloud);

	pcl2DMapFrame = temp_cloud;
	has_cloud = true;
}
