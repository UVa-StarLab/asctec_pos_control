#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <math.h>
#include <string.h>
#include <iostream>

#define freq 10.0

using namespace std;

bool isLanding = false;

ros::Publisher traj_pub, viz_goal;
ros::Subscriber land_sub;

pc_asctec_sim::pc_traj_cmd land_traj;

float land_x, land_y, land_z;
float land_x_p, land_y_p, land_z_p;
float land_vx, land_vy, land_vz;

float pre_x, pre_y, pre_z, pre_xv, pre_yv, pre_zv;
float t_all = 6;
float t_a = 2;
float t_b = 2;
float t_c = 2;

int main(int argc, char** argv) {
   
	ros::init(argc, argv, "land_predictor");
	ros::NodeHandle nh;

	string quad_name, land_frame, world_frame;
	ros::param::get("~name", quad_name);
	ros::param::get("~land_frame", land_frame);
	ros::param::get("~world_frame", world_frame);

	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points",10);

	viz_goal = nh.advertise<geometry_msgs::PointStamped>(quad_name + "/viz_goals",10);

	ros::Rate rate(freq);

	tf::StampedTransform transform;
	tf::TransformListener listener;

	ros::Time now = ros::Time(0); 
	ros::Time past = now;

        listener.waitForTransform(world_frame, land_frame, 
                             ros::Time(0), ros::Duration(3.0));
	while(ros::ok()) {
		ros::spinOnce();
		listener.lookupTransform(world_frame, land_frame, ros::Time(0), transform);
		
		now = transform.stamp_;
		double dt = now.toSec() - past.toSec() + (now.toNSec() - past.toNSec())/pow(10,10);
		past = now;	

		land_x = transform.getOrigin().x();
		land_y = transform.getOrigin().y();
		land_z = transform.getOrigin().z();

		cout << land_vx << endl;

		land_vx = (land_x - land_x_p) / dt;
		land_vy = (land_y - land_y_p) / dt;
		land_vz = (land_z - land_z_p) / dt;

		land_x_p = land_x;
		land_y_p = land_y;
		land_z_p = land_z;

		pre_x = land_x + land_vx*t_all;
		pre_y = land_y + land_vy*t_all;
		pre_z = land_z;

		geometry_msgs::PointStamped viz;
		viz.header.stamp = ros::Time::now();
		viz.header.frame_id = world_frame;
		viz.point.x = pre_x;
		viz.point.y = pre_y;
		viz.point.z = pre_z;

		viz_goal.publish(viz);
	
		rate.sleep();
	}
}
