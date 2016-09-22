#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <math.h>
#include <string.h>

using namespace std;

pc_asctec_sim::pc_goal_cmd goal;
string current_goal_id;
int counter;
int spins = 4*M_PI;

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
      ROS_INFO_STREAM("Spinning Now");
      pos_goal.publish(goal);
      current_goal_id = "Prep Point";

   }else if(current_goal_id == "Halt") {
      ROS_INFO_STREAM("Returning to 0,0,0.55");
      current_goal_id = "Shutdown1";
      goal.goal_id = current_goal_id;
      goal.z = 0.55;
      goal.z_vel = 0.0;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);

   }else if(current_goal_id == "Shutdown1") {
      ROS_INFO_STREAM("Returning to 0,0,0.55");
      current_goal_id = "Shutdown2";
      goal.goal_id = current_goal_id;
      goal.z = 0.55;
      goal.z_vel = 0.0;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);

   }else if(current_goal_id == "Shutdown2") {
      ROS_INFO_STREAM("Returning to 0,0,0.4");
      current_goal_id = "Shutdown3";
      goal.goal_id = current_goal_id;
      goal.z = 0.4;
      goal.z_vel = 0.0;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);
   
   }else if(current_goal_id == "Shutdown3") {
      ROS_INFO_STREAM("Returning to 0,0,0.25");
      current_goal_id = "Waiting";
      goal.goal_id = current_goal_id;
      goal.z = 0.25;
      goal.z_vel = 0.0;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);
   
   }else {
      ROS_INFO("Waiting, Manual Landing Required");
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;
   ros::Rate loop_rate = 1;

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>("/pos_goals", 10);
   goal_feedback = nh.subscribe("/goal_feedback", 1000, feedback_callback);

   counter = 0;
   goal.x = 0.0;
   goal.x_vel = 0.0;
   goal.y = 0.0;
   goal.y_vel = 0.0;
   goal.z = 0.7;
   goal.z_vel = 0.0;
   goal.yaw = 0.0;
   goal.yaw_vel = 0.0;
   goal.goal_limit = 0.01;
   current_goal_id = "Prep";

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: Yaw Spin");


   while(ros::Ok()) {
      ros::spinOnce();

      if(counter >= spins) {
          current_goal_id = "Halt";
          goal.goal_id = current_goal_id;
          goal.yaw = 0.0;
          goal.goal_limit = 0.01;
      }else {

          counter = counter + M_PI/8;
      }

      pos_goal.publish(goal);
      loop_rate.sleep();
   }
}
