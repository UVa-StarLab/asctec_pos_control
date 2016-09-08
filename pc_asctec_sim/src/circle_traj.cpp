#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <string.h>

using namespace std;

pc_asctec_sim::pc_goal_cmd goal;
string current_goal_id;
int counter;

float x_commands[8];
float y_commands[8];
float x_vel[8];
float y_vel[8];
string id[8];

ros::Publisher pos_goal;
ros::Subscriber goal_feedback;

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(current_goal_id == "Prep") {
      ROS_INFO_STREAM("Moving to start point");
      goal.x = 0.354;
      goal.x_vel = 0.0;
      goal.y = 0.354;
      goal.y_vel = 0.0;
      goal.goal_limit = 0.2;
      pos_goal.publish(goal);
      current_goal_id = "Prep Point";

   }else {
      ROS_INFO_STREAM("Goal ID: " + current_goal_id + " Reached");
      current_goal_id = id[counter];
      goal.goal_id = current_goal_id;
      goal.x = x_commands[counter];
      goal.x_vel = x_vel[counter];
      goal.y = y_commands[counter];
      goal.y_vel = y_vel[counter];
      pos_goal.publish(goal);
      ROS_INFO_STREAM("Goal ID: " + current_goal_id + " Set");
      counter++;
      counter &= 7;
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>("/pos_goals", 10);
   goal_feedback = nh.subscribe("/goal_feedback", 1000, feedback_callback);

   x_commands[0] = 0.0;
   x_commands[1] = -0.354;
   x_commands[2] = -0.5;
   x_commands[3] = -0.354;
   x_commands[4] = 0.0;
   x_commands[5] = 0.354;
   x_commands[6] = 0.5;
   x_commands[7] = 0.354;
   
   y_commands[0] = 0.5;
   y_commands[1] = 0.354;
   y_commands[2] = 0.0;
   y_commands[3] = -0.354;
   y_commands[4] = -0.5;
   y_commands[5] = -0.354;
   y_commands[6] = 0.0;
   y_commands[7] = 0.354;

   x_vel[0] = -0.3;
   x_vel[1] = -0.21;
   x_vel[2] = 0;
   x_vel[3] = 0.21;
   x_vel[4] = 0.3;
   x_vel[5] = 0.21;
   x_vel[6] = 0.0;
   x_vel[7] = -0.21;
   //x_vel[7] = 0.0;

   y_vel[0] = 0.0;
   y_vel[1] = -0.21;
   y_vel[2] = -0.3;
   y_vel[3] = -0.21;
   y_vel[4] = 0.0;
   y_vel[5] = 0.21;
   y_vel[6] = 0.3;
   y_vel[7] = 0.21;
   //y_vel[7] = 0.0;

   id[0] = "Point 0";
   id[1] = "Point 1";
   id[2] = "Point 2";
   id[3] = "Point 3";
   id[4] = "Point 4";
   id[5] = "Point 5";
   id[6] = "Point 6";
   id[7] = "Point 7";

   counter = 0;
   goal.z_vel = 0.0;
   goal.yaw_vel = 0.0;
   goal.z = 0.7;
   goal.yaw = 0.0;
   goal.goal_limit = 0.01;
   current_goal_id = "Prep";

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: Circle Trajectory");

   ros::spin();
}
