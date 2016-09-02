#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <string.h>

using namespace std;

pc_asctec_sim::pc_goal_cmd goal;
string current_goal_id;
float y_commands[8];
float x_commands[8];
string id[8];
int counter;

ros::Publisher pos_goal;
ros::Subscriber goal_feedback;

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   ROS_INFO_STREAM("Goal ID: " + current_goal_id + " Reached");
   current_goal_id = id[counter];
   goal.goal_id = current_goal_id;
   goal.x = x_commands[counter];
   goal.y = y_commands[counter];
   pos_goal.publish(goal);
   ROS_INFO_STREAM("Goal ID: " + current_goal_id + " Set");
   counter++;
   counter &= 3;
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>("/pos_goals", 10);
   goal_feedback = nh.subscribe("/goal_feedback", 1000, feedback_callback);
   
   y_commands[0] = 0.25;
   y_commands[1] = 0.25;
   y_commands[2] = -0.25;
   y_commands[3] = -0.25;
   y_commands[4] = 0.0;
   y_commands[5] = 0.0;
   y_commands[6] = 0.0;
   y_commands[7] = 0.0;

   x_commands[0] = 0.25;
   x_commands[1] = -0.25;
   x_commands[2] = -0.25;
   x_commands[3] = 0.25;
   x_commands[4] = 0.0;
   x_commands[5] = 0.0;
   x_commands[6] = 0.0;
   x_commands[7] = 0.0;

   id[0] = "Point 0";
   id[1] = "Point 1";
   id[2] = "Point 2";
   id[3] = "Point 3";
   id[4] = "Point 4";
   id[5] = "Point 5";
   id[6] = "Point 6";
   id[7] = "Point 7";

   counter = 0;
   goal.z = 1.0;
   goal.yaw = 0.0;
   goal.goal_limit = 0.05;
   current_goal_id = "Init";

   ros::spin();
}
