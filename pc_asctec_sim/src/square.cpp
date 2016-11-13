#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <string.h>
#include <sstream>

using namespace std;
int counter = 0;
int yaw_counter = 0;
float target_x [5] = {-0.6, -0.6, 0.6, 0.6, 0.0};
float target_y [5] = {0.6, -0.6, -0.6, 0.6, 0.0};
string base = "Square ";
string ID [5] = {"0", "1", "2", "3", "4"};
string target = base + ID[counter];
bool hover = true;
bool complete = false;
bool ready = false;
bool new_cmd = true;

pc_asctec_sim::pc_goal_cmd goal;
ros::Publisher pos_goal, quad_shutdown;
ros::Subscriber goal_feedback, tracker_shutdown, joy_call;

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg) 
{
   if(ready) {
      if(msg->buttons[2]) {
         hover = !hover;
	 if(hover) {
	    ROS_INFO("Hovering!");
	 }else {
	    ROS_INFO("Following trajectory!");
            goal.x = target_x[counter];
            goal.y = target_y[counter];
            goal.goal_id = base + ID[counter];
	    goal.wait_time = 3;
	    target = goal.goal_id;
	    new_cmd = true;
	 }
      }
   }
}

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(msg->goal_id == "Init") {
      goal.x = 0;
      goal.y = 0;
      goal.z = 0.7;
      goal.wait_time = 2;
      goal.goal_id = "Init1";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Init1") {
      goal.x = 0;
      goal.y = 0;
      goal.z = 0.8;
      goal.wait_time = 1;
      goal.goal_id = "Init2";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Init2") {
      goal.x = 0;
      goal.y = 0;
      goal.z = 0.9;
      goal.goal_id = "Init3";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Init3") {
      goal.x = 0;
      goal.y = 0;
      goal.z = 1.0;
      goal.goal_id = "Init4";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Init4") {
      ready = true;
      ROS_INFO("Waiting for Commands");
      goal.goal_id = "wait_for_button";

   }else if(msg->goal_id == target) {
      complete = true;
      counter++;
      counter = counter % 4;
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "Square_Trajectory");
   ros::NodeHandle nh;

   goal.x_vel = 0.0;
   goal.y_vel = 0.0;
   goal.z = 1.0;
   goal.z_vel = 0.0;
   goal.yaw = 0.0;
   goal.yaw_vel = 0.0;
   goal.goal_limit = 0.1;
   goal.wait_time = 3;

   tf::StampedTransform transform;
   tf::TransformListener listener;

   string quad_name;
   ros::param::get("~quad_name", quad_name);

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
   goal_feedback = nh.subscribe(quad_name + "/goal_feedback", 1000, feedback_callback);
   joy_call = nh.subscribe("/joy",10,joy_callback);
   listener.waitForTransform("/odom", "/vicon" + quad_name + quad_name, ros::Time(0), ros::Duration(5.0));

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: Square Trajectory");

   while(ros::ok()) {
      ros::spinOnce();

      if(ready) {
         if(!hover) {
            if(complete) {
               goal.wait_time = 3;
	       goal.goal_id = base + ID[counter];
	       target = base + ID[counter];
               goal.x = target_x[counter];
               goal.y = target_y[counter];
	       new_cmd = true;
	       complete = false;
            }
         }
	 if(new_cmd) {
            pos_goal.publish(goal);
	    new_cmd = false;
	 }
      }
   }
}
