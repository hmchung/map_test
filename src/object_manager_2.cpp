//Constraint on the number of object
//Constraint on distance between 2 similar objects
//Official queue
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

std::string node_name = "object_manage_2";

double distance_threshold = 10.0;

std::string topic_sub_marker = "/mixed_object_marker";
std::string topic_pub_marker_array = "/filtered_marker_array";
std::string topic_mission_1_stage = "/mission_1/status";

int count_constraints[5][7] = {
//	R 	G 	B 	BK 	W 	Y	O
	1,	1, 	1, 	0, 	0, 	0, 	0,  //Triangle
	1, 	1, 	1, 	0, 	0, 	0, 	0,  //Cruciform
	1, 	1, 	1, 	0, 	0, 	0, 	0,  //Circle
	2, 	2, 	0, 	1, 	2, 	0, 	0,  //Totem	
	0, 	0, 	0, 	0, 	0, 	0, 	0,  //Rectangle
};

void marker_cb(const visualization_msgs::Marker::ConstPtr& _marker);
bool processMarker(visualization_msgs::Marker _marker);
int countMarker(visualization_msgs::Marker _marker);
bool modifyMarker(visualization_msgs::Marker _marker, int target_index);
bool addNewMarker(visualization_msgs::Marker _marker);
bool getMarker(visualization_msgs::Marker _marker_type, int index, visualization_msgs::Marker *_marker_return);
bool overwriteCurrentMarker(visualization_msgs::Marker _marker, int index);

double getProjectedDistance(visualization_msgs::Marker _marker_1, visualization_msgs::Marker _marker_2);


visualization_msgs::MarkerArray filtered_marker_array;
visualization_msgs::Marker current_marker;

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	ros::Subscriber marker_sub = nh.subscribe<visualization_msgs::Marker>
		(topic_sub_marker, 0, marker_cb);

	ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>
		(topic_pub_marker_array, 10);

	ros::Rate loop_rate(1);
	int loop_count = 0;
	while (ros::ok()){
		marker_array_pub.publish(filtered_marker_array);

		if (loop_count % 5 == 0){
			int obj_count = filtered_marker_array.markers.size();
			// ROS_INFO("Number of objects: %d", obj_count);
			// for (int i = 0; i < obj_count; i++){
			// 	int temp_type = filtered_marker_array.markers[i].type;
			// 	int temp_color = filtered_marker_array.markers[i].id;
			// 	ROS_INFO("Obj type %d, color %d", temp_type, temp_color);
			// }
		}

		loop_count++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void marker_cb(const visualization_msgs::Marker::ConstPtr& _marker){
	visualization_msgs::Marker my_marker = *_marker;
	if(processMarker(my_marker)){

	}
}

bool processMarker(visualization_msgs::Marker _marker){
	//Distance constraint
	//Count constraint
	int new_marker_type = _marker.type;
	int new_marker_color = _marker.id;
	int new_marker_certainty = _marker.color.a;
	int marker_count_constraint = count_constraints[new_marker_type][new_marker_color];
	if (marker_count_constraint == 1){
		//Traverse to the right marker in queue and replace
		bool modify_success = modifyMarker(_marker, 1);
		//if (modify_success) ROS_INFO("HERE");
		//If not in array, add new
		if (!modify_success) addNewMarker(_marker);
		return true;
	} else {
		if (marker_count_constraint > 1){
			//Choose which one to replace
			int current_count = countMarker(_marker);
			if (current_count == 0) {
				addNewMarker(_marker);
			}
			if (current_count == 1) {
				visualization_msgs::Marker current_marker;
				bool canGetCurrentMarker = getMarker(_marker, 1, &current_marker);
				double new_marker_certainty = current_marker.color.a;
				if (new_marker_certainty == 1.0){ //new marker is laser
					if (new_marker_certainty == 1.0){ //current marker is also laser
						double distance = getProjectedDistance(_marker, current_marker);
						if (distance > distance_threshold) addNewMarker(_marker);
						else overwriteCurrentMarker(_marker, 1);
					} else { //current marker is vision
						overwriteCurrentMarker(_marker, 1);
					}
				} else { //new marker is vision
					if (new_marker_certainty == 1.0){ //current marker is laser
						addNewMarker(_marker);
					} else { //current marker is also vision
						overwriteCurrentMarker(_marker, 1);
					}
				}
			}
			if (current_count == 2) {
				//new marker
				double new_marker_certainty = _marker.color.a;

				//cerrent marker
				//The 1st one can only be laser
				//The 2nd one can be vision or laser
				visualization_msgs::Marker first_marker;
				getMarker(_marker, 1, &first_marker);
				double first_certainty = first_marker.color.a;
				visualization_msgs::Marker sencond_marker;
				getMarker(_marker, 2, &sencond_marker);
				double second_certainty = sencond_marker.color.a;
				if (second_certainty == 1.0){ //both markers are laser //replace the closer one
					if (new_marker_certainty == 1.0){ //only if the new marker is laser
						double distance_1 = getProjectedDistance(_marker, first_marker);
						double distance_2 = getProjectedDistance(_marker, sencond_marker);
						if (distance_1 > distance_2) overwriteCurrentMarker(_marker, 1);
						if (distance_2 > distance_1) overwriteCurrentMarker(_marker, 2);
					}
					// if the new marker is vision, just ignore
				} else { //the 2nd is vision
					overwriteCurrentMarker(_marker, 2);
				}			
			}
			if (current_count > 2) {
				ROS_ERROR("Run-time problem. Found more than 2 marker of the same type");
			}
		} else { // 0
			return false;
		}
	}
	return false;
}

bool modifyMarker(visualization_msgs::Marker _marker, int target_index){
	int target_type = _marker.type;
	int target_color = _marker.id;
	double target_certainty = _marker.color.a;
	for(int i = 1; i < filtered_marker_array.markers.size(); i++){
		int temp_obj_type = filtered_marker_array.markers[i].type;
		int temp_color_code = filtered_marker_array.markers[i].id;
		int temp_certainty = filtered_marker_array.markers[i].color.a;
		if (temp_obj_type == target_type && temp_color_code == target_color){
			if (target_certainty == 1.0){ 
				//New marker is from laser, replace old
				filtered_marker_array.markers[i].pose = _marker.pose;
				filtered_marker_array.markers[i].color.a = 1.0;
				// ROS_INFO("REPLACED WITH CERTAINTY");
			} else { 
				//New marker is pure vision
				//Only replace if old marker is also pure vision
				if (temp_certainty < 1.0){
					filtered_marker_array.markers[i].pose = _marker.pose;
					filtered_marker_array.markers[i].color.a = 0.3;
				}
			}
			return true;		
		}
	}
	return false;
}

bool addNewMarker(visualization_msgs::Marker _marker){
	filtered_marker_array.markers.push_back(_marker);
}

int countMarker(visualization_msgs::Marker _marker){
	int marker_count = 0;
	int target_type = _marker.type;
	int target_color = _marker.id;
	double target_certainty = _marker.color.a;
	for(int i = 1; i < filtered_marker_array.markers.size(); i++){
		int temp_obj_type = filtered_marker_array.markers[i].type;
		int temp_color_code = filtered_marker_array.markers[i].id;
		int temp_certainty = filtered_marker_array.markers[i].color.a;
		if (temp_obj_type == target_type && temp_color_code == target_color){
			marker_count += 1;	
		}
	}
	return marker_count;
}

bool getMarker(visualization_msgs::Marker _marker_type, int index, visualization_msgs::Marker *_marker_return){
	int marker_count = 0;
	int target_type = _marker_type.type;
	int target_color = _marker_type.id;
	for(int i = 1; i < filtered_marker_array.markers.size(); i++){
		int temp_obj_type = filtered_marker_array.markers[i].type;
		int temp_color_code = filtered_marker_array.markers[i].id;
		if (temp_obj_type == target_type && temp_color_code == target_color){
			marker_count += 1;
			*_marker_return = filtered_marker_array.markers[i];
			if 	(marker_count == index){
				return true;
			}
		}
	}
	return false;
}

bool overwriteCurrentMarker(visualization_msgs::Marker _marker_type, int index){
	int marker_count = 0;
	int target_type = _marker_type.type;
	int target_color = _marker_type.id;
	geometry_msgs::Pose target_pose = _marker_type.pose;
	double target_certainty = _marker_type.color.a;

	for(int i = 1; i < filtered_marker_array.markers.size(); i++){
		int temp_obj_type = filtered_marker_array.markers[i].type;
		int temp_color_code = filtered_marker_array.markers[i].id;
		if (temp_obj_type == target_type && temp_color_code == target_color){
			marker_count += 1;
			if 	(marker_count == index){
				filtered_marker_array.markers[i].pose = target_pose;
				filtered_marker_array.markers[i].color.a = target_certainty;
				return true;
			}
		}
	}
	return false;
}

double getProjectedDistance(visualization_msgs::Marker _marker_1, visualization_msgs::Marker _marker_2){
	geometry_msgs::Pose _pose_1 = _marker_1.pose;
	geometry_msgs::Pose _pose_2 = _marker_2.pose;
	double del_x = _pose_1.position.x - _pose_2.position.x;
	double del_y = _pose_1.position.y - _pose_2.position.y;
	double distance = sqrt(del_x * del_x - del_y * del_y);
	return distance;
}