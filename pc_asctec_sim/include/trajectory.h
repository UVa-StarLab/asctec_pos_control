#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#define freq 10.0
#define dt 1/freq

using namespace std;
using Eigen::MatrixXd;

int yaw_counter = 0;
bool waiting = true;
bool timing = false;

MatrixXd A(6,6);
MatrixXd B(24,8);
MatrixXd X(24,8);
MatrixXd t_data(2,8);

float c_time = 0.0;
int point_ct = 0;
int max_point = 0;

ros::Publisher pos_goal, viz_goal;
ros::Subscriber goal_feedback, joy_feed, traj_feed, state_feed;

pc_asctec_sim::pc_state state_data;
