#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <sensor_msgs/Joy.h>

#include <math.h>
#include <string.h>

#define repel_pts 3	//always odd
#define repel 0.7   //radius
#define repel_t 4.0

#define freq 10
#define maxV 0.2

using namespace std;

enum states {
waitStart,
waitHover,
waitA,
waitB
};

states state = waitStart;

bool isDone = true;
bool nextTraj = false;
bool isFlying = false;

string world, quad_name, obs_frame;
pc_asctec_sim::pc_state q_state;

ros::Publisher traj_pub, start_pub;
ros::Subscriber traj_sub, joy_sub, state_sub;

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	q_state = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]) {
		isFlying = !isFlying;

	}else if(msg->buttons[1]) {
		if(state != waitStart) {
			nextTraj = true;
		}
	}
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

float getTravTime(float x, float y, float z)
{
	float tTravel = sqrt(pow(q_state.x - x,2) + pow(q_state.y - y,2) + pow(q_state.z - z,2)) / maxV;
	return limit(tTravel, 8, 2);
}

void sendAvoidTrajectoryAB(float wait, float x, float y, float z, float yaw, tf::StampedTransform * transform)
{
	float obsy = transform->getOrigin().y();
	float obsx = transform->getOrigin().x();

	float t1 = getTravTime(0,sqrt(pow(repel,2) - pow(obsx,2)),1.0);
	float t2 = getTravTime(x,y,z) - t1;
	
	pc_asctec_sim::pc_traj_cmd cmd;
	for(int i=0; i<=repel_pts; i++) {
		float ay = sqrt(pow(repel,2) - pow(obsx,2));
		float y0 = (obsy-ay)*(1 - 2*i/repel_pts);
		float theta = acos(y0/repel);

		if(obsx > 0) {
			cmd.x[i+1] = obsx + repel*sin(theta);
			cmd.y[i+1] = obsy - repel*cos(theta);
			cmd.z[i+1] = 1.0;
			cmd.duration[i+1] = repel_t/repel_pts;

		}else {
			cmd.x[i+1] = obsx - repel*sin(theta);
			cmd.y[i+1] = obsy - repel*cos(theta);
			cmd.z[i+1] = 1.0;
			cmd.duration[i+1] = repel_t/repel_pts;

		}
	}
	cmd.x[repel_pts+2] = x;
	cmd.y[repel_pts+2] = y;
	cmd.z[repel_pts+2] = z;
	cmd.yaw[repel_pts+2] = yaw;
	cmd.duration[repel_pts+2] = t2;

	cmd.points = repel_pts+2;
	isDone = false;
	traj_pub.publish(cmd);
}

void sendAvoidTrajectoryBA(float wait, float x, float y, float z, float yaw, tf::StampedTransform * transform)
{
	float obsy = transform->getOrigin().y();
	float obsx = transform->getOrigin().x();

	float t1 = getTravTime(0,sqrt(pow(repel,2) - pow(obsx,2)),1.0);
	float t2 = getTravTime(x,y,z) - t1;

	pc_asctec_sim::pc_traj_cmd cmd;
	for(int i=0; i<=repel_pts; i++) {
		float ay = sqrt(pow(repel,2) - pow(obsx,2));
		float y0 = (obsy+ay)*(1 - 2*i/repel_pts);
		float theta = acos(y0/repel);

		if(obsx > 0) {
			cmd.x[i] = obsx + repel*sin(theta);
			cmd.y[i] = obsy + repel*cos(theta);
			cmd.z[i] = 1.0;
			cmd.duration[i] = repel_t/repel_pts;
		}else {
			cmd.x[i] = obsx - repel*sin(theta);
			cmd.y[i] = obsy + repel*cos(theta);
			cmd.z[i] = 1.0;
			cmd.duration[i] = repel_t/repel_pts;
		}
		
	}
	cmd.x[repel_pts+2] = x;
	cmd.y[repel_pts+2] = y;
	cmd.z[repel_pts+2] = z;
	cmd.yaw[repel_pts+2] = yaw;
	cmd.duration[repel_pts+2] = t2;

	cmd.points = repel_pts+2;
	isDone = false;
	traj_pub.publish(cmd);
}

void sendTrajectory(float wait, float x, float y, float z, float yaw) 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = x;
	cmd.y[0] = y;
	cmd.z[0] = z;
	cmd.yaw[0] = yaw;
	cmd.wait_time[0] = wait;
	cmd.duration[0] = getTravTime(x,y,z);
	cmd.points = 1;

	isDone = false;
	traj_pub.publish(cmd);
}

void sendLandTrajectory() 
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0.0;
	cmd.y[0] = 0.0;
	cmd.z[0] = 1.0;
	cmd.yaw[0] = 0.0;
	cmd.wait_time[0] = 0.5;
	cmd.duration[0] = getTravTime(0,0,1);

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = 0.0;
	cmd.yaw[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;

	isDone = false;
	traj_pub.publish(cmd);
}

bool obstacleExists(tf::StampedTransform * transform)
{
	/* Obstacle is observed if line traj intersects;
	 * obsx+rad > trajx (0) & obsx-rad < trajx (0)
	 */

	if(transform->getOrigin().x()+repel > 0 && transform->getOrigin().x()-repel < 0) {
		ROS_INFO("Obstacled detected, adapting trajectory");
		return true;
	}
	return false;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Obs_avoid");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;
   
	tf::StampedTransform transform;
	tf::TransformListener listener;

	ros::param::get("~w_frame", world);
	ros::param::get("~q_name", quad_name);
	ros::param::get("~obs_frame", obs_frame);

	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);
	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);
	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);
	
	listener.waitForTransform(world, obs_frame, ros::Time(0), ros::Duration(3.0));
	
	ROS_INFO("Running: Obstacle Avoider");

	while(ros::ok()) {

		ros::spinOnce();
		listener.lookupTransform(world, obs_frame, ros::Time(0), transform);

		switch(state) {
			case waitStart:
				if(isFlying) {
					ROS_INFO("Taking off!");
					state = waitHover;

					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);

					sendTrajectory(0,0,0,1,0);
				}
				break;

			case waitHover:
				if(isDone) {
					if(nextTraj) {
						ROS_INFO("Moving to point A");
						state = waitA;
						nextTraj = false;
						sendTrajectory(0,0,-1.5,1,0);
					}

					if(!isFlying) {
						ROS_INFO("Landing!");
						state = waitStart;

						sendLandTrajectory();
					}
				}
				break;

			case waitA:
				if(isDone) {
					if(nextTraj) {
						ROS_INFO("Observing obstacle(s) from A->B");
						nextTraj = false;
						state = waitB;

						if(obstacleExists(&transform)) {
							sendAvoidTrajectoryAB(0,0,1.5,1,0,&transform);
						}else {
							sendTrajectory(0,0,1.5,1,0);
						}
					}

					if(!isFlying) {
						ROS_INFO("Landing!");
						nextTraj = false;
						state = waitStart;

						sendLandTrajectory();
					}
				}
				break;

			case waitB:
				if(isDone) {
					if(nextTraj) {
						ROS_INFO("Observing obstacle(s) from B->A");
						nextTraj = false;
						state = waitA;

						if(obstacleExists(&transform)) {
							sendAvoidTrajectoryBA(0,0,-1.5,1,0,&transform);
						}else {
							sendTrajectory(0,0,-1.5,1,0);
						}
					}

					if(!isFlying) {
						ROS_INFO("Landing!");
						nextTraj = false;
						state = waitStart;

						sendLandTrajectory();
					}
				}
				break;
		}
	loop_rate.sleep();
	}

	return 0;
}