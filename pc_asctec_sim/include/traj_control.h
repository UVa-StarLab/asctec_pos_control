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
#define DEBUG 1                 //define to broadcast debug messages
#define CONTROL_RATE 50.0
#define dt 1/CONTROL_RATE
#define GOAL_LIMIT 0.01
#define INTEGRAL_LIMIT 14.0
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
void update_controller(struct PID_DATA * controller_ptr, 
                       struct POS_DATA * position_ptr);
void update_real_cmd(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr);
void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg);
bool goal_arrived(struct PID_DATA * pid_ptr, struct POS_DATA * pos_ptr);
float limit(float input, float ceiling);

/* --------------- Data Structure Definitions ------------ */

typedef struct PID_DATA
{
  float error_x;
  float error_x_vel;
  float integral_x;

  float error_y;
  float error_y_vel;
  float integral_y;

  float error_z;
  float error_z_vel;
  float integral_z;

  float error_yaw;
  float error_yaw_vel;
  float integral_yaw;

} pid_data;

typedef struct POS_DATA
{
  float pos_x;
  float pos_y;
  float pos_z;
  float pos_yaw;
  float relative_yaw;

  float pos_x_past;
  float pos_y_past;
  float pos_z_past;
  float pos_yaw_past;

  float vel_x;
  float vel_y;
  float vel_z;
  float vel_yaw;

  float goal_x;
  float goal_y;
  float goal_z;
  float goal_yaw;

  float goal_vel_x;
  float goal_vel_y;
  float goal_vel_z;
  float goal_vel_yaw;

  float goal_range;
  bool goal_arrival;
  string goal_id;

} pos_data;
