#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <math.h>
#include <string.h>

using namespace std;

ros::Publisher traj_pub;
pc_asctec_sim::pc_traj_cmd traj_cmd;

int main(int argc, char** argv) {
   
	ros::init(argc, argv, "pos_controller");
	ros::NodeHandle nh;

	string quad_name;
	ros::param::get("~name", quad_name);

	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);

	traj_cmd.z_end[0] = 0.7;
	traj_cmd.wait_time[0] = 2.5;
	traj_cmd.duration[0] = 5.0;

	traj_cmd.y_end[1] = -1.4;
	traj_cmd.y_v_end[1] = 0.5;
	traj_cmd.z_end[1] = 0.7;
	traj_cmd.wait_time[1] = 2.5;
	traj_cmd.duration[1] = 2.0;

	traj_cmd.y_end[2] = -0.5;
	traj_cmd.y_v_end[2] = 0.5;
	traj_cmd.z_end[2] = 1.1;
	traj_cmd.duration[2] = 2.0;

	traj_cmd.y_end[3] = 0.5;
	traj_cmd.y_v_end[3] = 0.5;
	traj_cmd.z_end[3] = 1.1;
	traj_cmd.duration[3] = 2.0;

	traj_cmd.y_end[4] = 1.4;
	traj_cmd.z_end[4] = 0.7;
	traj_cmd.duration[4] = 2.0;

	traj_cmd.points = 5;
	
	ros::Duration(3.0).sleep();
	ROS_INFO("Publishing test trajectory...");
	traj_pub.publish(traj_cmd);
	
	ros::spin();
	return 0;
}
