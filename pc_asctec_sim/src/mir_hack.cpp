#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pc_asctec_sim/pc_traj_cmd.h>

#include <sensor_msgs/Joy.h>

#include <math.h>
#include <string.h>

#define freq 5
#define maxV 0.3
#define maxV_T 0.7
#define TRIGGER_RANGE 0.2
#define F_HEIGHT 1.25
#define L_HEIGHT 0.75
#define REST 0.0

using namespace std;

int state = 0;
string count = "a";
bool flying = false;
bool over_box = false;
bool lower_box = false;
bool isDone = true;

float quad_x, quad_y, quad_z, quad_yaw;
float box_x, box_y;
float ugv_x, ugv_y;

string world, quad_name, quad_frame, box_frame, ugv_frame;

ros::Publisher traj_pub;
ros::Subscriber traj_sub, joy_sub;

void trajCallback(const std_msgs::Empty::ConstPtr& msg)
{
	isDone = true;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0] && state == 0) {
		flying = true;

	}else if(msg->buttons[0] && state == 1) {
		flying = false;

	}else if(msg->buttons[0] && state == 4) {
		flying = false;

	}else if(msg->buttons[1] && state == 1) {
		over_box = true;

	}else if(msg->buttons[1] && state == 2) {
		over_box = false;

	}else if(msg->buttons[2] && state == 2) {
		lower_box = true;

	}else if(msg->buttons[2] && state == 3) {
		lower_box = false;
	}	
}

void sendTrajectory(float time, float wait, float x, float y, float z, float yaw) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x_end[0] = x;
	cmd.y_end[0] = y;
	cmd.z_end[0] = z;
	cmd.yaw_end[0] = yaw;
	cmd.wait_time[0] = wait;
	cmd.duration[0] = time;

	cmd.points = 1;
	traj_pub.publish(cmd);
}

void sendLandTrajectory(float time) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x_end[0] = 0.0;
	cmd.y_end[0] = 0.0;
	cmd.z_end[0] = 1.0;
	cmd.yaw_end[0] = 0.0;
	cmd.wait_time[0] = 1.5;
	cmd.duration[0] = time;

	cmd.x_end[1] = 0.0;
	cmd.y_end[1] = 0.0;
	cmd.z_end[1] = 0.0;
	cmd.yaw_end[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;
	traj_pub.publish(cmd);
}

float limit(float value, float upper, float lower)
{
	float temp = value;
	if(value > upper) {
		temp = upper;

	}else if(value < lower) {
		temp = lower;
	}

	return temp;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "Joy Tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
   
	tf::StampedTransform transform;
	tf::TransformListener listener;

	ros::param::get("~world_frame", world);
	ros::param::get("~quad_name", quad_name);
	ros::param::get("~ugv_frame", ugv_frame);
	ros::param::get("~quad_frame", quad_frame);
	ros::param::get("~box_frame", box_frame);

	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);

	//listener.waitForTransform(world, quad_frame, ros::Time(0), ros::Duration(3.0));
	//listener.waitForTransform(world, box_frame, ros::Time(0), ros::Duration(3.0));

	listener.lookupTransform(world, box_frame, ros::Time(0), transform);
	box_x = transform.getOrigin().x();
	box_y = transform.getOrigin().y();

	ROS_INFO("Running: Joy Tracker");

	while(ros::ok()) {
		ros::spinOnce();

		listener.lookupTransform(world, quad_frame, ros::Time(0), transform);
		quad_x = transform.getOrigin().x();
		quad_y = transform.getOrigin().y();
		quad_z = transform.getOrigin().z();

		listener.lookupTransform(world, ugv_frame, ros::Time(0), transform);
		ugv_x = transform.getOrigin().x();
		ugv_y = transform.getOrigin().y();
		
		switch(state) {
			case 0:
				// Transition fwd - Waits for joy signal "flying" = true, moves quad + mirror to start height
				if(isDone && flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, 0.0, 0.0, F_HEIGHT, 0.0);

					ROS_INFO("Taking off to 0.0, 0.0, %f, time of travel: %f", F_HEIGHT, tTravel);
					state = 1;
				}
				break;

			case 1:
				// Transition fwd - Waits for joy signal "over_box" = true, moves quad + mirror to box
				if(isDone && over_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, box_x, box_y, F_HEIGHT, 0.0);

					ROS_INFO("Hovering over target, time of travel: %f", tTravel);
					state = 2;

				// Transition bwd - Waits for joy signal "flying" = false, moves quad + mirror to resting height
				}else if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((REST - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, 0.0, 0.0, REST, 0.0);

					ROS_INFO("Returning to rest, time of travel: %f", tTravel);
					state = 0;
				}
				break;

			case 2:
				// Transition fwd - Waits for joy signal "lower_box" = true, moves quad + mirror down to hide box
				if(isDone && lower_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((L_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, box_x, box_y, L_HEIGHT, 0.0);

					ROS_INFO("Setting up mirror demo...");
					state = 3;

				// Transition bwd - Waits for joy signal "over_box" = false, moves quad + mirror back to center
				}else if(isDone && !over_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, 0, 0, F_HEIGHT, 0.0);
				
					ROS_INFO("Returning to center...");
					state = 1;
				}
				break;

			case 3:
				// Transition fwd - Waits for trigger signal and reveals obstacle
				if(isDone) {
					float trigger = sqrt(pow((box_x - quad_x),2) + pow((box_y - quad_y),2));
					if(trigger < TRIGGER_RANGE) {
						float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV_T;
						tTravel = limit(tTravel, 10, 0.25);
						sendTrajectory(tTravel, 0.0, box_x, box_y, F_HEIGHT, 0.0);

						ROS_INFO("Triggered, revealing obstacle!");
						state = 4;
					}

				// Transition bwd - Waits for joy signal "lower_box" = false, moves back up
				}else if(isDone && !lower_box) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((F_HEIGHT - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 0.0, box_x, box_y, F_HEIGHT, 0.0);

					ROS_INFO("Joy reset, revealing obstacle...");
					state = 2;
				}
				break;	
	
			case 4:
				// Transition bwd - Waits for reset signal "flying" = false, move back to center
				if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((REST - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendLandTrajectory(tTravel);

					ROS_INFO("Landing at center...");
					state = 0;
				}
		}
		loop_rate.sleep();
	}
}
