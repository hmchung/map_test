#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <queue>

std::string node_name = "object_manager";

std::string pub_name_map = "map";

//map properties
const int map_width = 2000; // cells
const int map_height = 2000; // cells
float map_resolution = 0.05; // m/cell
int map_data[map_width][map_height];

float map_origin_position_x = 0.0;
float map_origin_position_y = 0.0;
float map_origin_position_z = 0.0;

class Small_Object{
public:
	enum {
		BOY_BLK, BOY_WHI, BOY_RED, BOY_GRN, BOY_BLU,
		MRK_RND_BLK, MRK_RND_WHI, MRK_RND_RED, MRK_RND_GRN, MRK_RND_BLU,
		MRK_SQR_BLK, MRK_SQR_WHI, MRK_SQR_RED, MRK_SQR_GRN, MRK_SQR_BLU,
		MRK_TRI_BLK, MRK_TRI_WHI, MRK_TRI_RED, MRK_TRI_GRN, MRK_TRI_BLU,
		MRK_CRF_BLK, MRK_CRF_WHI, MRK_CRF_RED, MRK_CRF_GRN, MRK_CRF_BLU
	};
	std::string my_type;
	float my_size;
	geometry_msgs::Point my_position;

	Small_Object(std::string _name, float _size, float _x, float _y, float _z);
};

Small_Object::Small_Object(std::string _type, float _size, float _x, float _y, float _z){
	my_type = _type;
	my_size = _size;
	my_position.x = _x;
	my_position.y = _y;
	my_position.z = _z;
}


int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	//Object queue
	std::queue<Small_Object> object_queue;
	std::queue<Small_Object> object_queue_swap;

	//dummy objects
	Small_Object obj_1 = Small_Object("BOY_BLK", 1.0, 2.0, 3.0, 0.0);
	Small_Object obj_2 = Small_Object("BOY_BLK", 2.0, 4.0, 6.0, 0.0);
	object_queue.push(obj_1);
	object_queue.push(obj_2);
	
	//Subscriber

	//Publisher
	ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>(pub_name_map, 10);
	
	ros::Rate loop_rate(30);
	while (ros::ok()){
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

		//Draw objects
		//for (int obj_index = 0; obj_index < object_queue.size(); obj_index++){
		while(!object_queue.empty()){
			Small_Object temp_object = object_queue.front();
			object_queue.pop();
			object_queue_swap.push(temp_object);
			//dummy object
			float object_x = temp_object.my_position.x;
			float object_y = temp_object.my_position.y;
			float object_d = temp_object.my_size;

			for (int j = 0; j < map_height; j++){
				for (int i = 0; i < map_width; i++){
					float temp_dx = i * map_resolution - object_x;
					float temp_dy = j * map_resolution - object_y;
					float temp_distance = sqrt(temp_dx * temp_dx + temp_dy * temp_dy);
					if (temp_distance < object_d)  map_data[i][j] = 100;
				}
			}
			//ROS_INFO("Object: %d", obj_index);
		}

		//Recover object queue
		//for (int obj_index = 0; obj_index < object_queue_swap.size(); obj_index++){
		while(!object_queue_swap.empty()){
			Small_Object temp_object = object_queue_swap.front();
			object_queue_swap.pop();
			object_queue.push(temp_object);
		}

		//Put data into map
		for (int j = 0; j < map_height; j++){
			for (int i = 0; i < map_width; i++){
				my_map.data.push_back(map_data[i][j]);
			}
		}

		pub_map.publish(my_map);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

