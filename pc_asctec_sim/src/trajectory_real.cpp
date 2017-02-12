#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#define freq 20.0
#define dt 1/freq

using namespace std;
using Eigen::MatrixXd;

int yaw_counter = 0;
bool waiting = true;
bool timing = false;

MatrixXd A(6,6);
MatrixXd B(24,24);
MatrixXd X(24,24);
MatrixXd t_data(2,24);

double c_time = 0.0;
double range = 0.0001;
int point_ct = 0;
int max_point = 0;

string quad_name, world;

ros::Publisher pos_goal, traj_goal, viz_goal, trail_pub;
ros::Subscriber goal_feedback, traj_feed, state_feed;

pc_asctec_sim::pc_state state_data;

visualization_msgs::Marker trail;

void init_A(void)
{
	for(int i = 0; i < 6; i++) {
		for(int k = 0; k < 6; k++) {
			A(i,k) = 0.0;
		}
	}

	A(0,5) = 1;
	A(2,4) = 1;
	A(4,3) = 2; 
}

void calc_A(float time) 
{
	for(int i=5; i>=0; i--) {
		A(1,5-i) = pow(time,i);
		A(3,5-i) = i*pow(time,i-1);
		A(5,5-i) = i*(i-1)*pow(time,i-2);
	}
}

void init_trail(void)
{
	trail.header.frame_id = world;
	trail.header.stamp = ros::Time::now();
	trail.id = 2;
	trail.action = visualization_msgs::Marker::ADD;
	trail.type = visualization_msgs::Marker::LINE_LIST;
	trail.color.a = 1.0;
	trail.color.g = 1.0;				
	trail.color.b = 1.0;

	trail.scale.x = 0.05;
	trail.scale.y = 0.05;

	geometry_msgs::Point vis_trail;
	vis_trail.x = state_data.x;
	vis_trail.y = state_data.y;
	vis_trail.z = state_data.z;

	trail.points.push_back(vis_trail);
}

void publish_trail(float x, float y, float z)
{
	geometry_msgs::Point vis_trail;
	vis_trail.x = x;
	vis_trail.y = y;
	vis_trail.z = z;

	trail.points.push_back(vis_trail);
	trail_pub.publish(trail);
	trail.points.push_back(vis_trail);
}

void timerCallback(const ros::TimerEvent&) {
	ROS_INFO("Point %i timer expired", point_ct);
	point_ct++;
	c_time = 0.0;
	timing = false;
}

void traj_callback(const pc_asctec_sim::pc_traj_cmd::ConstPtr& msg) 
{
	if(waiting) {
		if(msg->points != 0) {
			max_point = msg->points;
			for(int i = 0; i < max_point; i++) {
				B(1,i) = msg->x[i];
				B(3,i) = msg->vx[i];
				B(5,i) = msg->ax[i];
	
				B(7,i) = msg->y[i];
				B(9,i) = msg->vy[i];
				B(11,i) = msg->ay[i];

				B(13,i) = msg->z[i];
				B(15,i) = msg->vz[i];
				B(17,i) = msg->az[i];

				B(19,i) = msg->yaw[i];
				B(21,i) = msg->vyaw[i];
				B(23,i) = msg->ayaw[i];

				t_data(0,i) = msg->wait_time[i];
				t_data(1,i) = msg->duration[i];

			}
			B(0,0) = state_data.x;
			B(2,0) = state_data.vx;
			B(4,0) = state_data.ax;
	
			B(6,0) = state_data.y;
			B(8,0) = state_data.vy;
			B(10,0) = state_data.ay;
	
			B(12,0) = state_data.z;
			B(14,0) = state_data.vz;
			B(16,0) = state_data.az;
		
			B(18,0) = state_data.yaw;
			B(20,0) = state_data.vyaw;
			B(22,0) = state_data.ayaw;
	
			for(int i = 1; i < max_point; i++) {
				for(int j = 0; j < 4; j++) {
					B(6*j,i) = B(6*j+1,i-1);
					B(6*j+2,i) = B(6*j+3,i-1);
					B(6*j+4,i) = B(6*j+5,i-1);      
				}
			} 
		
			point_ct = 0.0;
			c_time = 0.0;
			ROS_INFO("New Trajectory with %i points heard!", msg->points);
			ros::Time t_s = ros::Time::now();
			for(int i = 0; i < max_point; i++) {
				calc_A(t_data(1,i));
				for(int k = 0; k < 4; k++) {
					X.block<6,1>(6*k,i) = A.colPivHouseholderQr().solve(B.block<6,1>(6*k,i));
				}
			}
			ros::Time t_e = ros::Time::now();
			float t_f = ((t_e - t_s).toNSec());
			ROS_INFO("Trajectory calculation took %f ms", t_f/1000000);
			waiting = false;
		}else {
			ROS_INFO("Error, trajectory has 0 points. Msg ignored");
		}
	}else {
		ROS_INFO("Trajectory already in motion! Msg ignored");
	}
}

void state_callback(const pc_asctec_sim::pc_state::ConstPtr& msg) 
{
	state_data = *msg;
}

int main(int argc, char** argv) {
   
	ros::init(argc, argv, "pos_controller");
	ros::NodeHandle nh;
	ros::Timer timer_event = nh.createTimer(ros::Duration(3.0), timerCallback, true);
	timer_event.stop();

	ros::param::get("~name", quad_name);
	ros::param::get("~world", world);
	pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
	viz_goal = nh.advertise<geometry_msgs::PointStamped>(quad_name + "/viz_goals",10);
	trail_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/trajectory_trail",10);

	traj_goal = nh.advertise<std_msgs::Empty>(quad_name + "/traj_end",10);
	state_feed = nh.subscribe(quad_name + "/state", 10, state_callback);
	traj_feed = nh.subscribe(quad_name + "/traj_points", 10, traj_callback);
	ros::Rate rate(freq);

	init_A();
	init_trail();
	while (ros::ok()) {
		ros::spinOnce();
		if(point_ct == max_point && !waiting) {
			ROS_INFO("Trajectory Completed!");
			std_msgs::Empty fin;
			traj_goal.publish(fin);

			c_time = 0.0;
			waiting = true;
		}
		if(!waiting) {
			pc_asctec_sim::pc_goal_cmd goal; 

			if(!timing) {
				c_time += dt;
			}

			goal.goal_id = c_time;

			for(int i=5; i>=0; i--) {
				goal.x += X(5-i,point_ct)*pow(c_time,i);
				goal.vx += i*X(5-i,point_ct)*pow(c_time,i-1);
				goal.ax += i*(i-1)*X(5-i,point_ct)*pow(c_time,i-2);

				goal.y += X(11-i,point_ct)*pow(c_time,i);
				goal.vy += i*X(11-i,point_ct)*pow(c_time,i-1);
				goal.ay += i*(i-1)*X(11-i,point_ct)*pow(c_time,i-2);

				goal.z += X(17-i,point_ct)*pow(c_time,i);
				goal.vz += i*X(17-i,point_ct)*pow(c_time,i-1);
				goal.az += i*(i-1)*X(17-i,point_ct)*pow(c_time,i-2);

				goal.yaw += X(23-i,point_ct)*pow(c_time,i);
				goal.vyaw += i*X(23-i,point_ct)*pow(c_time,i-1);
				goal.ayaw += i*(i-1)*X(23-i,point_ct)*pow(c_time,i-2);
			}

			geometry_msgs::PointStamped viz;
			viz.header.stamp = ros::Time::now();
			viz.header.frame_id = "/odom";
			viz.point.x = goal.x;
			viz.point.y = goal.y;
			viz.point.z = goal.z;

			publish_trail(goal.x, goal.y, goal.z);
			pos_goal.publish(goal);
			viz_goal.publish(viz);

			if((c_time + range) >= t_data(1,point_ct)) {
				if(t_data(0,point_ct) != 0.0 && !timing) {
					timing = true;
					timer_event.setPeriod(ros::Duration(t_data(0,point_ct)), true);
					ROS_INFO("Point %i reached, waiting for %f seconds", point_ct, t_data(0,point_ct));
					timer_event.start();

				}else if(t_data(0,point_ct) == 0.0 && !timing){
					ROS_INFO("Point %i reached", point_ct);
					point_ct++;
					c_time = 0.0;
					timer_event.stop();
				}
			}
		}
		rate.sleep();
	}
	return 0;
}
