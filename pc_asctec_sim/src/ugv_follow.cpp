#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <math.h>
#include <string.h>

#define freq 10.0
#define dt 1/freq
#define TIMEOUT 0
#define ROBOT_OFFSET 1.2
#define ERROR_RANGE 0.05

using namespace std;

pc_asctec_sim::pc_goal_cmd goal;
int counter;
int timer = 100;
bool tracking = false;

ros::Publisher pos_goal, quad_shutdown;
ros::Subscriber goal_feedback, tracker_shutdown;

void shutdown_callback(const std_msgs::Empty::ConstPtr& msg)
{
   ROS_INFO("Landing Quadrotor...");
   std_msgs::Empty stop;
   quad_shutdown.publish(stop);
   tracking = false;
}

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(msg->goal_id == "Init") {
      ROS_INFO("Moving to 0,0,0.7");
      goal.x = 0.0;
      goal.y = 0.0;
      goal.z = 0.7;
      goal.goal_id = "Prep0";
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);
   
   }else if(msg->goal_id == "Prep0") {
      ROS_INFO_STREAM("Moving to 0,0,0.8");
      goal.goal_id = "Prep1";
      goal.z = 0.8;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Prep1") {
      ROS_INFO_STREAM("Moving to 0,0,0.9");
      goal.goal_id = "Prep2";
      goal.z = 0.9;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Prep2") {
      ROS_INFO_STREAM("Moving to 0,0,1.0");
      goal.goal_id = "Prep3";
      goal.z = 1.0;
      goal.goal_limit = 0.05;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Prep3") {
      ROS_INFO("Begin Tracking Target");
      tracking = true;
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;
   ros::Rate loop_rate = freq;
   
   counter = 0;
   goal.x_vel = 0.0;
   goal.y_vel = 0.0;
   goal.z_vel = 0.0;
   goal.yaw = 0.0;
   goal.yaw_vel = 0.0;

   tf::StampedTransform transform;
   tf::TransformListener listener;

   string world, ugv, quad_name;
   ros::param::get("~world_frame", world);
   ros::param::get("~ugv_frame", ugv);
   ros::param::get("~quad_name", quad_name);

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
   quad_shutdown = nh.advertise<std_msgs::Empty>(quad_name + "/shutdown", 10);
   goal_feedback = nh.subscribe(quad_name + "/goal_feedback", 1000, feedback_callback);
   tracker_shutdown = nh.subscribe("/ugv_follow/shutdown", 1, shutdown_callback);

   listener.waitForTransform(world, ugv, ros::Time(0), ros::Duration(10.0));
   listener.waitForTransform(world, "/vicon" + quad_name + quad_name, ros::Time(0), ros::Duration(10.0));
   float robot_x, robot_y, robot_yaw, error;

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: UGV Follow");

   while(ros::ok()) {
      ros::spinOnce();
      listener.lookupTransform(world, ugv, ros::Time(0), transform);

      if(tracking) {
         robot_x = transform.getOrigin().x();
	 robot_y = transform.getOrigin().y();
         robot_yaw = tf::getYaw(transform.getRotation());
	 error = sqrt(pow((robot_x - goal.x),2) + pow((robot_y - goal.y),2));

         if(error >= ERROR_RANGE) {
            goal.x = robot_x + ROBOT_OFFSET*cos(robot_yaw);
            goal.y = robot_y + ROBOT_OFFSET*sin(robot_yaw);
            goal.yaw = robot_yaw;
            goal.goal_id = counter;
            counter++;
	 }
         pos_goal.publish(goal);
      }
      loop_rate.sleep();
   }
}
