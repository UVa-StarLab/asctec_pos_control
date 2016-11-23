#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <math.h>
#include <string.h>

#define freq 20.0
#define dt 1/freq
#define joy_gain 0.3
#define DEAD 0.0

using namespace std;

pc_asctec_sim::pc_goal_cmd goal;
int counter;
int yaw_counter = 0;
bool tracking = false;
bool new_joy = false;

float robot_x, robot_y, robot_z, robot_yaw;
float joy_x, joy_y, joy_z, joy_yaw;

ros::Publisher pos_goal, quad_shutdown;
ros::Subscriber goal_feedback, tracker_shutdown, joy_feed;

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg) {
   if(msg->buttons[5]) {
	 new_joy = false;

      if(abs(msg->axes[1]) >= DEAD) {
         joy_z = msg->axes[1];
	 ROS_INFO("joy_z: %f", joy_z);
	 new_joy = true;

      }else {
	 joy_z = 0.0;
      }

      if(abs(msg->axes[0]) >= DEAD) {
         joy_yaw = msg->axes[0];
	 ROS_INFO("joy_yaw: %f", joy_yaw);
	 new_joy = true;

      }else {
	 joy_yaw = 0.0;
      }

      if(abs(msg->axes[4]) >= DEAD) {
         joy_x = msg->axes[4];
	 ROS_INFO("joy_x: %f", joy_x);
	 new_joy = true;

      }else {
	 joy_x = 0.0;
      }

      if(abs(msg->axes[3]) >= DEAD) {
         joy_y = msg->axes[3];
	 ROS_INFO("joy_y: %f", joy_y);
	 new_joy = true;

      }else {
	 joy_y = 0.0;
      }

   }else {
      joy_x = 0;
      joy_y = 0;
      joy_z = 0;
      joy_yaw = 0;
      new_joy = false;
   }
}

void feedback_callback(const pc_asctec_sim::pc_feedback::ConstPtr& msg)
{  
   if(msg->goal_id == "Init") {
      ROS_INFO("0.7 m reached");
      tracking = true;
   }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "joy_node");
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

   string world, quad_name;
   ros::param::get("~world_frame", world);
   ros::param::get("~quad_name", quad_name);

   pos_goal = nh.advertise<pc_asctec_sim::pc_goal_cmd>(quad_name + "/pos_goals", 10);
   goal_feedback = nh.subscribe(quad_name + "/goal_feedback", 1000, feedback_callback);
   joy_feed = nh.subscribe("/joy", 1, joy_callback);

   listener.waitForTransform(world, "/vicon" + quad_name + quad_name, ros::Time(0), ros::Duration(10.0));
   bool check = false;
   float robot_yaw_past;

   ROS_INFO("Waypoint Server Initialized");
   ROS_INFO("Running: Joy Follow");

   while(ros::ok()) {
      ros::spinOnce();
      listener.lookupTransform(world, "/vicon" + quad_name + quad_name, ros::Time(0), transform);
      robot_x = transform.getOrigin().x();
      robot_y = transform.getOrigin().y();
      robot_z = transform.getOrigin().z();

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
      if(tracking && new_joy) {
         goal.x = robot_x + joy_gain * joy_x;
         goal.y = robot_y + joy_gain * joy_y;
         goal.yaw = robot_yaw + 2*M_PI*yaw_counter + joy_gain * joy_yaw;
	 goal.z = robot_z + joy_gain * joy_z;
         goal.goal_id = counter;
         goal.wait_time = 0;
         counter++;
         pos_goal.publish(goal);
      }

      loop_rate.sleep();
   }
}
