#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <sensor_msgs/Joy.h>

#include <math.h>
#include <string.h>

#define freq 10
#define maxV 0.25
#define maxVZ 0.2

using namespace std;

int state = 0;
bool isDone = true;
bool isStarted = false;
string quad_name;
pc_asctec_sim::pc_state quad_state;

ros::Publisher traj_pub, start_pub;
ros::Subscriber traj_sub, state_sub, joy_sub;

void stateCallback(const pc_asctec_sim::pc_state::ConstPtr& msg)
{
	quad_state = *msg;
}

void trajCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isDone = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(msg->buttons[0]) {
		isStarted = !isStarted;
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

void sendRiseTrajectory(float x, float y, float z, float yaw, float wait) 
{
	float tTravel = sqrt(pow(x - quad_state.x,2) + pow(y - quad_state.y,2) + pow((z - quad_state.z),2)) / maxVZ;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = x;
	cmd.y[0] = y;
	cmd.z[0] = z;
	cmd.yaw[0] = yaw;
	cmd.wait_time[0] = wait;
	cmd.duration[0] = tTravel;

	cmd.points = 1;
	traj_pub.publish(cmd);
	isDone = false;
}

void sendPosTestTrajectory() 
{
	float tTravel = sqrt(pow(-0.5 - quad_state.x,2) + pow(quad_state.y,2) + pow((1 - quad_state.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = -0.5;
	cmd.y[0] = 0;
	cmd.vy[0] = maxV;
	cmd.z[0] = 1;
	cmd.yaw[0] = 0;
	cmd.duration[0] = tTravel;

	cmd.x[1] = 0;
	cmd.y[1] = 1;
	cmd.z[1] = 1;
	cmd.yaw[1] = 0;
	cmd.wait_time[1] = 3.0;
	cmd.duration[1] = tTravel;

	cmd.points = 2;
	traj_pub.publish(cmd);
	isDone = false;
}

void sendNegTestTrajectory() 
{
	float tTravel = sqrt(pow(-0.5 - quad_state.x,2) + pow(quad_state.y,2) + pow((1 - quad_state.z),2)) / maxV;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = -0.5;
	cmd.y[0] = 0;
	cmd.vy[0] = -maxV;
	cmd.z[0] = 1;
	cmd.yaw[0] = 0;
	cmd.duration[0] = tTravel;

	cmd.x[1] = 0;
	cmd.y[1] = -1;
	cmd.z[1] = 1;
	cmd.yaw[1] = 0;
	cmd.wait_time[1] = 3.0;
	cmd.duration[1] = tTravel;

	cmd.points = 2;
	traj_pub.publish(cmd);
	isDone = false;
}

void sendYawTest(bool pos)
{
	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0;
	cmd.y[0] = 0;
	cmd.z[0] = 1;
	cmd.duration[0] = 4;

	if(pos) {
		ROS_INFO("Sending pi/2");
		cmd.yaw[0] = M_PI/2;

	}else {
		ROS_INFO("Sending -pi/2");
		cmd.yaw[0] = -M_PI/2;

	}

	cmd.points = 1;
	traj_pub.publish(cmd);
	isDone = false;
}

void sendLandTrajectory() 
{
	float tTravel = sqrt(pow(quad_state.x,2) + pow(quad_state.y,2) + pow((1 - quad_state.z),2)) / maxVZ;
	tTravel = limit(tTravel, 10, 1);

	pc_asctec_sim::pc_traj_cmd cmd;
	cmd.x[0] = 0.0;
	cmd.y[0] = 0.0;
	cmd.z[0] = 1.0;
	cmd.yaw[0] = 0.0;
	cmd.wait_time[0] = 0.5;
	cmd.duration[0] = tTravel;

	cmd.x[1] = 0.0;
	cmd.y[1] = 0.0;
	cmd.z[1] = 0.0;
	cmd.yaw[1] = 0.0;
	cmd.wait_time[1] = 0.0;
	cmd.duration[1] = 3;

	cmd.points = 2;
	traj_pub.publish(cmd);
	isDone = false;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "Tuner");
	ros::NodeHandle nh;
	ros::Rate loop_rate = freq;

	ros::param::get("~q_name", quad_name);

	start_pub = nh.advertise<std_msgs::Bool>(quad_name + "/start", 10);
	traj_pub = nh.advertise<pc_asctec_sim::pc_traj_cmd>(quad_name + "/traj_points", 10);

	traj_sub = nh.subscribe(quad_name + "/traj_end", 10, trajCallback);
	state_sub = nh.subscribe(quad_name + "/state", 10, stateCallback);
	joy_sub = nh.subscribe("/joy", 10, joyCallback);

	ROS_INFO("Running: Param Tuner");

	while(ros::ok()) {
		loop_rate.sleep();
		ros::spinOnce();

		switch(state) {
			case 0:
				//Fly to 0,0,1
				//Check exit conditions
				if(isDone && isStarted) {
					ROS_INFO("Started!");
					std_msgs::Bool start;
					start.data = true;
					start_pub.publish(start);
					sendRiseTrajectory(0,0,1,0,1.0);
					state = 1;
				}
				break;

			case 1:
				//Fly to 0,1,1
				//Check exit conditions
				if(isDone && !isStarted) {
					sendLandTrajectory();
					state = 0;

				}else if(isDone && isStarted) {
					sendYawTest(true);
					state = 2;
				}
				break;

			case 2:
				//Fly to 0,-1,1
				//Check exit conditions
				if(isDone && !isStarted) {
					sendLandTrajectory();
					state = 0;

				}else if(isDone && isStarted) {
					sendYawTest(false);
					state = 1;
				}
				break;

		}
	}
}
