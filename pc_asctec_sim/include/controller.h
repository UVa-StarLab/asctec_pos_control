#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <pc_asctec_sim/SICmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <string.h>
#include <math.h>

#define REAL  1			//define to broadcast to asctec hummingbird quadrotor
#define DEBUG 0                 //define to broadcast debug messages
#define CONTROL_RATE 50.0
#define dt 1/CONTROL_RATE
#define GOAL_LIMIT 0.01
#define INTEGRAL_LIMIT 4.0
#define TRAJ_SIZE 4

#define THRUST_MAX 5.0
#define THRUST_MIN 0.0

#define ROLL_MAX 8.0
#define ROLL_MIN -ROLL_MAX

#define PITCH_MAX 8.0
#define PITCH_MIN -PITCH_MAX

#define YAW_MAX 10.0
#define YAW_MIN -YAW_MAX

using namespace std;

/* ---------------- Prototype Definitions ---------------- */

void init(struct POS_DATA * pos_ptr, 
          struct PID_DATA * ctl_ptr);

void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg);

bool goal_arrived(struct PID_DATA * pid_ptr, struct POS_DATA * pos_ptr);

void update_controller(struct PID_DATA * controller_ptr, 
                       struct POS_DATA * position_ptr);

void update_sim_cmd(struct PID_DATA * ctl_ptr);

void update_real_cmd(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr);

/* --------------- Data Structure Definitions ------------ */

typedef struct PID_DATA
{
  float error_x;
  float error_x_past;
  float integral_x;
  float diff_x;
  float error_accel_x;

  float error_y;
  float error_y_past;
  float integral_y;
  float diff_y;
  float error_accel_y;

  float error_z;
  float error_z_past;
  float integral_z;
  float diff_z;
  float error_accel_z;

  float error_yaw;
  float error_yaw_past;
  float integral_yaw;
  float diff_yaw;
  float error_accel_yaw;

} pid_data;

typedef struct POS_DATA
{
  float pos_x;
  float pos_y;
  float pos_z;
  float pos_yaw;

  float pos_x_past;
  float pos_y_past;
  float pos_z_past;
  float pos_yaw_past;

  float vel_x;
  float vel_y;
  float vel_z;
  float vel_yaw;

  float vel_x_past;
  float vel_y_past;
  float vel_z_past;
  float vel_yaw_past;

  float accel_x;
  float accel_y;
  float accel_z;
  float accel_yaw;

  float goal_x;
  float goal_y;
  float goal_z;
  float goal_yaw;

  float goal_accel_x;
  float goal_accel_y;
  float goal_accel_z;
  float goal_accel_yaw;

  float goal_range;
  bool goal_arrival;
  string goal_id;

} pos_data;
