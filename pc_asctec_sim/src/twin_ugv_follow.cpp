#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <math.h>
#include <string.h>

#define freq 10.0
#define dt 1/freq
#define TIMEOUT 0
#define ROBOT_OFFSET 1.0
#define ERROR_RANGE 0.15

using namespace std;

pc_asctec_sim::pc_goal_cmd h1_goal;
pc_asctec_sim::pc_goal_cmd h2_goal;

int counter;
float h1_startx, h1_starty, h2_startx, h2_starty;
bool tracking1 = false;
bool tracking2 = false;
bool halt = false;

ros::Publisher pos_goal_h1, pos_goal_h2, shutdown_h1, shutdown_h2;
ros::Subscriber goal_feedback_h1, goal_feedback_h2, shutdown_request;


void shutdown_callback(const std_msgs::Empty::ConstPtr& msg)
{
   ROS_INFO("Shutting Down UGV follow...");
   tracking1 = false;
   tracking2 = false;
   std_msgs::Empty stop;
   shutdown_h1.publish(stop);
   shutdown_h2.publish(stop);
   halt = true;
}

void H1feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(msg->goal_id == "Init") {
      ROS_INFO("Moving to 0,0,0.7");
      h1_goal.x = h1_startx;
      h1_goal.y = h1_starty;
      h1_goal.yaw = 0.0;
      h1_goal.z = 0.7;
      h1_goal.goal_id = "Prep0";
      h1_goal.goal_limit = 0.05;
      pos_goal_h1.publish(h1_goal);
   
   }else if(msg->goal_id == "Prep0") {
      ROS_INFO_STREAM("Moving to 0,0,0.8");
      h1_goal.goal_id = "Prep1";
      h1_goal.z = 0.8;
      h1_goal.goal_limit = 0.05;
      pos_goal_h1.publish(h1_goal);

   }else if(msg->goal_id == "Prep1") {
      ROS_INFO_STREAM("Moving to 0,0,0.9");
      h1_goal.goal_id = "Prep2";
      h1_goal.z = 0.9;
      h1_goal.goal_limit = 0.05;
      pos_goal_h1.publish(h1_goal);

   }else if(msg->goal_id == "Prep2") {
      ROS_INFO_STREAM("Moving to 0,0,1.0");
      h1_goal.goal_id = "Prep3";
      h1_goal.z = 1.0;
      h1_goal.goal_limit = 0.05;
      pos_goal_h1.publish(h1_goal);

   }else if(msg->goal_id == "Prep3") {
      ROS_INFO("quad1 ready to track target");
      tracking1 = true;
   }
}

void H2feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(msg->goal_id == "Init") {
      ROS_INFO("Moving to 0,0,0.7");
      h2_goal.x = h2_startx;
      h2_goal.y = h2_starty;
      h2_goal.yaw = 0.0;
      h2_goal.z = 0.7;
      h2_goal.goal_id = "Prep0";
      h2_goal.goal_limit = 0.05;
      pos_goal_h2.publish(h2_goal);
   
   }else if(msg->goal_id == "Prep0") {
      ROS_INFO_STREAM("Moving to 0,0,0.8");
      h2_goal.goal_id = "Prep1";
      h2_goal.z = 0.8;
      h2_goal.goal_limit = 0.05;
      pos_goal_h2.publish(h2_goal);

   }else if(msg->goal_id == "Prep1") {
      ROS_INFO_STREAM("Moving to 0,0,0.9");
      h2_goal.goal_id = "Prep2";
      h2_goal.z = 0.9;
      h2_goal.goal_limit = 0.05;
      pos_goal_h2.publish(h2_goal);

   }else if(msg->goal_id == "Prep2") {
      ROS_INFO_STREAM("Moving to 0,0,1.0");
      h2_goal.goal_id = "Prep3";
      h2_goal.z = 1.0;
      h2_goal.goal_limit = 0.05;
      pos_goal_h2.publish(h2_goal);

   }else if(msg->goal_id == "Prep3") {
      ROS_INFO("quad2 ready to track target");
      tracking2 = true;
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "pos_test");
   ros::NodeHandle nh;
   ros::Rate loop_rate = freq;
   counter = 0;

   tf::StampedTransform transform;
   tf::TransformListener listener;

   string world, ugv, h1_name, h2_name;
   ros::param::get("~world_frame", world);
   ros::param::get("~ugv_frame", ugv);
   ros::param::get("~h1_name", h1_name);
   ros::param::get("~h2_name", h2_name);

   pos_goal_h1 = nh.advertise<pc_asctec_sim::pc_goal_cmd>(h1_name + "/pos_goals", 10);
   goal_feedback_h1 = nh.subscribe(h1_name + "/goal_feedback", 1000, H1feedback_callback);

   pos_goal_h2 = nh.advertise<pc_asctec_sim::pc_goal_cmd>(h2_name + "/pos_goals", 10);
   goal_feedback_h2 = nh.subscribe(h2_name + "/goal_feedback", 1000, H2feedback_callback);

   shutdown_h1 = nh.advertise<std_msgs::Empty>(h1_name + "/shutdown", 10);
   shutdown_h2 = nh.advertise<std_msgs::Empty>(h2_name + "/shutdown", 10);
   shutdown_request = nh.subscribe("/twin_ugv_follow/shutdown", 1, shutdown_callback);

   listener.waitForTransform(world, ugv, ros::Time(0), ros::Duration(10.0));
   string vicon_quad = "/vicon" + h1_name + h1_name;
   listener.waitForTransform(world, vicon_quad, ros::Time(0), ros::Duration(10.0));
   vicon_quad = "/vicon" + h2_name + h2_name;
   listener.waitForTransform(world, vicon_quad, ros::Time(0), ros::Duration(10.0));

   float robot_x, robot_y, robot_yaw, error1, error2;
   vicon_quad = "/vicon" + h1_name + h1_name;
   listener.lookupTransform(world, vicon_quad, ros::Time(0), transform);
   h1_startx = transform.getOrigin().x();
   h1_starty = transform.getOrigin().y();

   vicon_quad = "/vicon" + h2_name + h2_name;
   listener.lookupTransform(world, vicon_quad, ros::Time(0), transform);
   h2_startx = transform.getOrigin().x();
   h2_starty = transform.getOrigin().y();

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: UGV Follow...");

   while(ros::ok()) {
      ros::spinOnce();
      listener.lookupTransform(world, ugv, ros::Time(0), transform);

      if(tracking1 && tracking2) {
         robot_x = transform.getOrigin().x();
	 robot_y = transform.getOrigin().y();
         robot_yaw = tf::getYaw(transform.getRotation());

	 error1 = sqrt(pow((robot_x - h1_goal.x),2) + pow((robot_y - h1_goal.y),2));
	 error2 = sqrt(pow((robot_x - h2_goal.x),2) + pow((robot_y - h2_goal.y),2));

         if(error1 >= ERROR_RANGE) {
            h1_goal.x = robot_x - ROBOT_OFFSET*sin(robot_yaw + M_PI/2);
            h1_goal.y = robot_y + ROBOT_OFFSET*cos(robot_yaw + M_PI/2);
            h1_goal.yaw = robot_yaw + M_PI/2;
            h1_goal.goal_id = counter;
            h1_goal.wait_time = 0;
	 }

         if(error2 >= ERROR_RANGE) {
            h2_goal.x = robot_x - ROBOT_OFFSET*sin(robot_yaw - M_PI/2);
            h2_goal.y = robot_y + ROBOT_OFFSET*cos(robot_yaw - M_PI/2);
            h2_goal.yaw = robot_yaw + M_PI/2;
            h2_goal.goal_id = counter;
            h2_goal.wait_time = 0;
	 }
         pos_goal_h1.publish(h1_goal);
	 pos_goal_h2.publish(h2_goal);
         counter++;
      }
      loop_rate.sleep();
   }
}
