#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <math.h>
#include <string.h>

using namespace std;
#define freq 5.0
#define dt 1/freq

pc_asctec_sim::pc_goal_cmd goal;
string current_goal_id;
float y_commands[16];
float y_vel[16];
float y_acc[16];

string id[16];
int counter;
float c_time = 0.0;
bool start_traj = false;

ros::Publisher pos_goal;
ros::Subscriber goal_feedback;

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(current_goal_id == "Init") {
      ROS_INFO_STREAM("Moving to start point");
      goal.x = 0.0;
      goal.x_vel = 0.0;
      goal.x_acc = 0.0;

      goal.y = -2.0;
      goal.y_vel = 0.0;
      goal.y_acc = 0.0;

      goal.goal_limit = 0.01;
      goal.wait_time = 3.0;
      pos_goal.publish(goal);
      current_goal_id = "Prep Point";

   }else if(current_goal_id == "Prep Point") {
      start_traj = true;

   }/*else {
      ROS_INFO_STREAM("Goal ID: " + current_goal_id + " Reached");
      current_goal_id = id[counter];
      goal.goal_id = current_goal_id;

      goal.y = y_commands[counter];
      goal.y_vel = y_vel[counter];
      goal.y_acc = y_acc[counter];

      goal.goal_limit = 0.1;
      goal.wait_time = 0.0;
      pos_goal.publish(goal);
      ROS_INFO_STREAM("Goal ID: " + current_goal_id + " Set");
      if(counter != 15) {
         counter++;
      }
   }*/
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>("/hummingbird_1/pos_goals", 10);
   goal_feedback = nh.subscribe("/hummingbird_1/goal_feedback", 1000, feedback_callback);
   
   y_commands[0] = -2.0;
   y_commands[1] = -1.7;
   y_commands[2] = -1.5;
   y_commands[3] = -1.3;
   y_commands[4] = -1.1;
   y_commands[5] = -0.9;
   y_commands[6] = -0.6;
   y_commands[7] = -0.4;
   y_commands[8] = -0.2;
   y_commands[9] = 0.1;
   y_commands[10] = 0.3;
   y_commands[11] = 0.6;
   y_commands[12] = 0.8;
   y_commands[13] = 1.1;
   y_commands[14] = 1.3;
   y_commands[15] = 1.6;

   y_vel[0] = 0.1;
   y_vel[1] = 0.2;
   y_vel[2] = 0.35;
   y_vel[3] = 0.4;
   y_vel[4] = 0.5;
   y_vel[5] = 0.6;
   y_vel[6] = 0.6;
   y_vel[7] = 0.6;
   y_vel[8] = 0.6;
   y_vel[9] = 0.6;
   y_vel[10] = 0.5;
   y_vel[11] = 0.4;
   y_vel[12] = 0.35;
   y_vel[13] = 0.2;
   y_vel[14] = 0.1;
   y_vel[15] = 0.0;

   y_acc[0] = 0.1;
   y_acc[1] = 0.1;
   y_acc[2] = 0.1;
   y_acc[3] = 0.1;
   y_acc[4] = 0.1;
   y_acc[5] = 0.0;
   y_acc[6] = 0.0;
   y_acc[7] = 0.0;
   y_acc[8] = 0.0;
   y_acc[9] = 0.0;
   y_acc[10] = -0.1;
   y_acc[11] = -0.1;
   y_acc[12] = -0.1;
   y_acc[13] = -0.1;
   y_acc[14] = -0.1;
   y_acc[15] = 0.0;

   id[0] = "Point 0";
   id[1] = "Point 1";
   id[2] = "Point 2";
   id[3] = "Point 3";
   id[4] = "Point 4";
   id[5] = "Point 5";
   id[6] = "Point 6";
   id[7] = "Point 7";
   id[8] = "Point 8";
   id[9] = "Point 9";
   id[10] = "Point 10";
   id[11] = "Point 11";
   id[12] = "Point 12";
   id[13] = "Point 13";
   id[14] = "Point 14";
   id[15] = "Point 15";

   counter = 0;
   goal.x_vel = 0.0;
   goal.z_vel = 0.0;
   goal.z_acc = 0.0;

   goal.yaw_vel = 0.0;
   goal.yaw_acc = 0.0;

   goal.x = 0.0;
   goal.z = 0.7;
   goal.yaw = 0.0;
   goal.goal_limit = 0.01;
   current_goal_id = "Init";

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: Trapezoidal Line Trajectory");
   ROS_INFO("Running for 4 second trajectory");

   ros::Rate loop_rate = freq;

   float C5 = 0.0211;
   float C4 = -0.2109;
   float C3 = 0.5625;
   float C2 = 0.0;
   float C1 = 0.0;
   float C0 = -2.0;

   while(ros::ok()) {
      ros::spinOnce();
      
      if(start_traj) {
         c_time += dt;

         goal.y = C5 * pow(c_time,5) + C4 * pow(c_time,4) + C3 * pow(c_time,3) + C2 * pow(c_time,2) + C1 * c_time + C0;
         goal.y_vel = 5 * C5 * pow(c_time,4) + 4 * C4 * pow(c_time,3) + 3 * C3 * pow(c_time,2) + 2 * C2 * c_time + C1;
         goal.y_acc = 20 * C5 * pow(c_time,3) + 12 * C4 * pow(c_time,2) + 6 * C3 * c_time + 2 * C2;
         goal.wait_time = 0.0;
         goal.goal_id = c_time;
         pos_goal.publish(goal);
         if(c_time >= 4.0) {
	    ros::spin();
         }
      }
      loop_rate.sleep();
   }
}
