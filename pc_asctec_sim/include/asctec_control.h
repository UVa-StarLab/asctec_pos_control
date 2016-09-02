#ifndef ASCTECCONTROL_H
#define ASCTECCONTROL_H

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <pc_asctec_sim/SICmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <string.h>
#include <math.h>

using namespace std;

#define REAL 1			//define to broadcast to asctec hummingbird quadrotor
#define CONTROL_RATE 50.0
#define dt 1/CONTROL_RATE
#define TRAJ_SIZE 4

#define THRUST_MAX 5.0
#define THRUST_MIN 0.0

#define ROLL_MAX 8.0
#define ROLL_MIN -ROLL_MAX

#define PITCH_MAX 8.0
#define PITCH_MIN -PITCH_MAX

#define YAW_MAX 10.0
#define YAW_MIN -YAW_MAX

class AscTecController
{
  public:

 /* --------------- Data Structure Definitions ------------ */

   typedef struct PID_DATA
   {
     float error_x_cur;
     float error_x_past;
     float integral_x;
     float diff_x;

     float error_y_cur;
     float error_y_past;
     float integral_y;
     float diff_y;

     float error_z_cur;
     float error_z_past;
     float integral_z;
     float diff_z;

     float error_yaw_cur;
     float error_yaw_past;
     float integral_yaw;
     float diff_yaw;

   } pid_data;

   typedef struct POS_DATA
   {
     float pos_x;
     float pos_y;
     float pos_z;
     float pos_yaw;

     float goal_x;
     float goal_y;
     float goal_z;
     float goal_yaw;
     float goal_range;

     bool goal_arrived;
   } pos_data;

   typedef struct GOAL_DATA
   {
     float goal_x;
     float goal_y;
     float goal_z;
     float goal_yaw;
     float goal_range;
     bool goal_arrival;

   } goal_data;

   typedef struct TRAJ_DATA
   {
     short current_point; 
     bool completed;

     float x_traj[TRAJ_SIZE];
     float y_traj[TRAJ_SIZE];
     float z_traj[TRAJ_SIZE];
     float yaw_traj[TRAJ_SIZE];
     float radius_traj[TRAJ_SIZE];

   } traj_data;

   typedef struct PARAM_DATA
   {
     float k_p_x;
     float k_i_x;
     float k_d_x;

     float k_p_y;
     float k_i_y;
     float k_d_y;

     float k_p_z;
     float k_i_z;
     float k_d_z;

     float k_p_yaw;
     float k_i_yaw;
     float k_d_yaw;

   } param_data;

   string global_frame;
   string asctec_frame;

   float k_p_x;
   float k_i_x;
   float k_d_x;

   float k_p_y;
   float k_i_y;
   float k_d_y;

   float k_p_z;
   float k_i_z;
   float k_d_z;

   float k_p_yaw;
   float k_i_yaw;
   float k_d_yaw;

   PID_DATA ctl_data;
   AscTecController::POS_DATA position_data;
   AscTecController::TRAJ_DATA trajectory_data;

   quadrotor_msgs::PositionCommand sim_cmd;	
   pc_asctec_sim::SICmd real_cmd;
   
   /* ---------------- Prototype Definitions ---------------- */

   AscTecController();
   ~AscTecController();

   void init(void);

   void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg);

   bool goal_arrived(void);

   bool update_goal(void);

   void update_controller(void);
 
   void update_position(float x, float y, float z, float yaw);

   void update_sim_cmd(void);

   void update_real_cmd(void);

   void set_frames(string world_frame, string quad_frame);

   void set_parameters(AscTecController::PARAM_DATA * param_ptr);

} asctecController;
#endif
