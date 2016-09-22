#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <string.h>

#define freq 5.0
#define dt 1/freq
#define TIMEOUT 0

using namespace std;

pc_asctec_sim::pc_goal_cmd goal;
int counter;
int timer = 100;
bool tracking = false;

ros::Publisher pos_goal;
ros::Subscriber goal_feedback;

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(msg->goal_id == "Init") {
      ROS_INFO("Moving to 0,0,0.8");
      goal.x = 0.0;
      goal.y = 0.0;
      goal.z = 0.8;
      goal.goal_id = "Prep1";
      goal.goal_limit = 0.01;
      pos_goal.publish(goal);
   
   }else if(msg->goal_id == "Prep1") {
      ROS_INFO_STREAM("Returning to 0,0,0.9");
      goal.goal_id = "Prep2";
      goal.z = 0.9;
      goal.goal_limit = 0.01;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Prep2") {
      ROS_INFO_STREAM("Returning to 0,0,1.0");
      goal.goal_id = "Prep3";
      goal.z = 1.0;
      goal.goal_limit = 0.01;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Prep3") {
      ROS_INFO("Begin Tracking Jackal1");
      tracking = true;

   }else if(msg->goal_id == "Shutdown1") {
      ROS_INFO_STREAM("Returning to 0,0,0.8");
      goal.goal_id = "Shutdown2";
      goal.z = 0.8;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Shutdown2") {
      ROS_INFO_STREAM("Returning to 0,0,0.7");
      goal.goal_id = "Shutdown3";
      goal.z = 0.7;
      pos_goal.publish(goal);
   
   }else if(msg->goal_id == "Shutdown3") {
      ROS_INFO_STREAM("Returning to 0,0,0.6");
      goal.goal_id = "Shutdown4";
      goal.z = 0.6;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Shutdown4") {
      ROS_INFO_STREAM("Returning to 0,0,0.5");
      goal.goal_id = "Shutdown5";
      goal.z = 0.5;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Shutdown5") {
      ROS_INFO_STREAM("Returning to 0,0,0.4");
      goal.goal_id = "Shutdown6";
      goal.z = 0.4;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Shutdown6") {
      ROS_INFO_STREAM("Returning to 0,0,0.3");
      goal.goal_id = "Shutdown7";
      goal.z = 0.3;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Shutdown7") {
      ROS_INFO_STREAM("Returning to 0,0,0.2");
      goal.goal_id = "Waiting";
      goal.z = 0.2;
      pos_goal.publish(goal);
   
   }else if(msg->goal_id == "Waiting") {
      ROS_INFO("Waiting, Manual Landing Required");
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;
   ros::Rate loop_rate = freq;

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>("/pos_goals", 10);
   goal_feedback = nh.subscribe("/goal_feedback", 1000, feedback_callback);
   
   counter = 0;
   goal.x_vel = 0.0;
   goal.y_vel = 0.0;
   goal.z_vel = 0.0;
   goal.yaw = 0.0;
   goal.yaw_vel = 0.0;

   tf::StampedTransform transform;
   tf::TransformListener listener;

   listener.waitForTransform("/odom", "/vicon/jackal1/jackal1", 
                             ros::Time(0), ros::Duration(10.0));
   ROS_INFO("Jackal Transform found!");

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: Jackal1 Follow");

   while(ros::ok()) {
      ros::spinOnce();

      listener.lookupTransform("/odom", "/vicon/jackal1/jackal1", 
                               ros::Time(0), transform);
      if(tracking) {
         goal.x = transform.getOrigin().x();
         goal.y = transform.getOrigin().y();
         goal.goal_id = counter;
         counter++;

      #if TIMEOUT
      if(counter >= timer) {
         tracking = false;
         goal.x = 0.0;
         goal.y = 0.0;
         goal.goal_limit = 0.05;
         goal.goal_id = "Shutdown1";
      }
      if(counter % 50 == 0) {
         ROS_INFO("10 Seconds Remaining");
      }
      #endif
         pos_goal.publish(goal);
      }
      loop_rate.sleep();
   }
}