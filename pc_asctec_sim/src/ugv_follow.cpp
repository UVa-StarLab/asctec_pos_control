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
int yaw_counter = 0;
int timer = 100;
bool tracking = true;
float robot_x, robot_y, robot_yaw;

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
      ROS_INFO("Move Towards Target");
      goal.x = robot_x + ROBOT_OFFSET*cos(robot_yaw);
      goal.y = robot_y + ROBOT_OFFSET*sin(robot_yaw);
      goal.z = 1.0;
      goal.yaw = robot_yaw + M_PI;
      goal.goal_id = "Lower0";
      goal.goal_limit = 0.1;
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Lower0") {
      ROS_INFO("Lowering Mirror to 0.9...");
      goal.z = 0.9;
      goal.goal_id = "Lower1";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Lower1") {
      ROS_INFO("Lowering Mirror to 0.75...");
      goal.z = 0.75;
      goal.goal_id = "Lower2";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Lower2") {
      ROS_INFO("Lowering Mirror to 0.65...");
      goal.z = 0.6;
      goal.goal_id = "Lower3";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Lower3") {
      ROS_INFO("Lowering Mirror to 0.5...");
      goal.z = 0.5;
      goal.goal_id = "Track";
      pos_goal.publish(goal);

   }else if(msg->goal_id == "Track") {
      ROS_INFO("Begin tracking target");
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
   quad_shutdown = nh.advertise<std_msgs::Empty>(quad_name + "/shutdown_center", 10);
   goal_feedback = nh.subscribe(quad_name + "/goal_feedback", 1000, feedback_callback);
   tracker_shutdown = nh.subscribe("/ugv_follow/shutdown", 1, shutdown_callback);

   listener.waitForTransform(world, ugv, ros::Time(0), ros::Duration(10.0));
   listener.waitForTransform(world, "/vicon" + quad_name + quad_name, ros::Time(0), ros::Duration(10.0));
   
   bool check = false;
   float robot_yaw_past;
   listener.lookupTransform(world, ugv, ros::Time(0), transform);
   robot_x = transform.getOrigin().x();
   robot_y = transform.getOrigin().y();
   robot_yaw = tf::getYaw(transform.getRotation()) + 2*M_PI*yaw_counter;

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: UGV Follow");

   while(ros::ok()) {
      ros::spinOnce();
      listener.lookupTransform(world, ugv, ros::Time(0), transform);
      robot_x = transform.getOrigin().x();
      robot_y = transform.getOrigin().y();
      robot_yaw_past = robot_yaw;
      robot_yaw = tf::getYaw(transform.getRotation()) + 2*M_PI*yaw_counter;
      
      float yaw_dif = robot_yaw - robot_yaw_past;
      if(abs(yaw_dif) > M_PI) {
	 if(!check) {
            if(yaw_dif < 0.0) {
	       yaw_counter += 1;
	    }else {
	       yaw_counter -= 1;
	    }
	    check = true;
	 }else {
	    check = false;
	 }
      }
      
      if(tracking) {
         goal.x = robot_x + ROBOT_OFFSET*cos(robot_yaw);
         goal.y = robot_y + ROBOT_OFFSET*sin(robot_yaw);
         goal.yaw = robot_yaw + 3*M_PI/4;
         goal.goal_id = counter;
         counter++;
         pos_goal.publish(goal);
      }
      loop_rate.sleep();
   }
}
