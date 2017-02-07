#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>

#include <sensor_msgs/Joy.h>

#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#define freq 5
#define maxV 0.2
#define XBOUND_H 1.0
#define XBOUND_L -1.0
#define YBOUND_H 1.5
#define YBOUND_L -1.5

#define YW_GAIN 0.3
#define X_GAIN 0.3
#define Y_GAIN 0.3
#define Z_GAIN 0.15

using namespace std;

int state = 0;
string count = "a";
bool flying = false;
bool tracking = false;
bool isDone = true;

float quad_x, quad_y, quad_z, quad_yaw;
float joy_x, joy_y, joy_z, joy_yaw;
string world, quad_name, quad_frame;

ros::Publisher traj_pub, border_pub, pos_pub, start_pub;
ros::Subscriber traj_sub, joy_sub;

void trajCallback(const std_msgs::Empty::ConstPtr& msg)
{
	isDone = true;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[2] && (state == 2 || state == 3)) {
		tracking = !tracking;

	}else if(msg->buttons[3] && (state == 0 || state == 2)) {
		flying = !flying;

	}else if(msg->buttons[5] && state == 3) {
		joy_yaw = 0;
		joy_x = 0;
		joy_y = 0;
		if(abs(msg->axes[1]) >= 0.3) {
			joy_z = Z_GAIN * msg->axes[1];
		}else {
			joy_z = 0.0;
		}

	}else if(msg->buttons[4] && state == 3) {
                joy_z = 0.0;
                if(msg->axes[4] >= 0.3 || msg->axes[4] <= -0.3) {
                        joy_x = X_GAIN * msg->axes[4];
                }else {
                        joy_x = 0.0;
                }
                if(msg->axes[3] >= 0.3 || msg->axes[3] <= -0.3) {
                        joy_y = Y_GAIN * msg->axes[3];  
                }else {
                        joy_y = 0.0;
                }
                if(msg->axes[0] >= 0.3 || msg->axes[0] <= -0.3) {
                        joy_yaw = YW_GAIN * msg->axes[0];
                }else {
                        joy_yaw = 0.0;
                }
	}else if(state == 3) {
                joy_x = 0.0;
                joy_y = 0.0;
                joy_z = 0.0;
                joy_yaw = 0.0;
	}
}

void show_border(bool outOf)
{
	visualization_msgs::Marker border;
	geometry_msgs::Point corner;

	border.header.frame_id = world;
	border.header.stamp = ros::Time::now();
	border.id = 2;
	border.action = visualization_msgs::Marker::ADD;
	border.type = visualization_msgs::Marker::LINE_LIST;

	if(outOf) {
		border.color.a = 1.0;
		border.color.r = 1.0;	
	}else {
		border.color.a = 1.0;
		border.color.g = 1.0;	
	}
			
	border.scale.x = 0.05;
	border.scale.y = 1.0;
	border.scale.z = 1.0;

	corner.x = XBOUND_L;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_H;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_H;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	corner.x = XBOUND_L;
	corner.y = YBOUND_L;	
	corner.z = 1.0;
	border.points.push_back(corner);

	border_pub.publish(border);
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

void sendPoint(float x, float y, float z, float yaw)
{
	pc_asctec_sim::pc_goal_cmd next;
	next.x = x;
	next.y = y;
	next.z = z;
	next.yaw = yaw;
	pos_pub.publish(next);
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

bool outBorder(void)
{
	bool temp = false;
	if(quad_x > XBOUND_H || quad_x < XBOUND_L || quad_y > YBOUND_H || quad_y < YBOUND_L) {
		temp = true;
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
	ros::param::get("~quad_frame", quad_frame);

	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	pos_pub = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
	border_pub = nh.advertise<visualization_msgs::Marker>(quad_name + "/border", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);

	listener.waitForTransform(world, quad_frame, ros::Time(0), ros::Duration(3.0));

	ROS_INFO("Running: Joy Tracker");

	while(ros::ok()) {
		ros::spinOnce();

		listener.lookupTransform(world, quad_frame, ros::Time(0), transform);
		quad_x = transform.getOrigin().x();
		quad_y = transform.getOrigin().y();
		quad_z = transform.getOrigin().z();

		show_border(outBorder());

		switch(state) {
			case 0:
				/* Basic waiting state, waits for start signal from controller
				*  Input: bool flying
				*/

				//Check exit conditions
				if(flying && isDone) {
					state = 1;
				}
				break;

			case 1:
				/* Fly to Hovering at 0, 0, 1
				*  Input: state change
				*/

				//Check exit conditions
				if(isDone) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow((1 - quad_z),2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendTrajectory(tTravel, 2.0, 0.0, 0.0, 1.0, 0.0);
					isDone = false;

					ROS_INFO("Taking off to 0.0, 0.0, 1.0, time of travel: %f", tTravel);
					state = 2;
				}
				break;

			case 2:
				/* Hold state, hovers in place
				*  Input: state change + trajectory completion
				*  Output: stop tracking
				*/

				//Check exit conditions
				if(isDone && !flying) {
					float tTravel = sqrt(pow(quad_x,2) + pow(quad_y,2) + pow(quad_z,2)) / maxV;
					tTravel = limit(tTravel, 10, 1);
					sendLandTrajectory(tTravel);
					isDone = false;

					ROS_INFO("Landing at 0.0, 0.0, 0.0, time of travel: %f", tTravel);
					state = 0;

				}else if(tracking && isDone) {
					ROS_INFO("Joy Control Started!");
					state = 3;
				}
				break;

			case 3:
				/* Tracking state, follows joystick
				*  Input: state change
				*/

				if(isDone) {
					if(abs(joy_x) > 0.1 || abs(joy_y) > 0.1 || abs(joy_z) > 0.1 || abs(joy_yaw) > 0.1) {
						float xNew = quad_x + joy_x;
						float yNew = quad_y + joy_y;
						float zNew = quad_z + joy_z;
						float yawNew = quad_yaw + joy_yaw;

						xNew = limit(xNew, XBOUND_H, XBOUND_L);
						yNew = limit(yNew, YBOUND_H, YBOUND_L);
						sendPoint(xNew, yNew, zNew, yawNew);
					}
				}

				//Check exit conditions
				if(!tracking) {
					ROS_INFO("Halting Joy Control");		
					state = 2;
				}
				break;		
		}
		loop_rate.sleep();
	}
}
