#include <asctec_control.h>

AscTecController::AscTecController()
{
   init();
   return;
}

AscTecController::~AscTecController()
{
   return;
}

void AscTecController::init(void)
{
   ctl_data.error_x_cur = 0.0;
   ctl_data.error_x_past = 0.0;  
   ctl_data.integral_x = 0.0;
   ctl_data.diff_x = 0.0;

   ctl_data.error_y_cur = 0.0;
   ctl_data.error_y_past = 0.0;  
   ctl_data.integral_y = 0.0;
   ctl_data.diff_y = 0.0;

   ctl_data.error_z_cur = 0.0;
   ctl_data.error_z_past = 0.0;  
   ctl_data.integral_z = 0.0;
   ctl_data.diff_z = 0.0;

   ctl_data.error_yaw_cur = 0.0;
   ctl_data.error_yaw_past = 0.0;  
   ctl_data.integral_yaw = 0.0;
   ctl_data.diff_yaw = 0.0;

   //initialize a trajectory

   trajectory_data.x_traj[0] = 0.0;
   trajectory_data.x_traj[1] = 0.0;
   trajectory_data.x_traj[2] = 0.0;
   trajectory_data.x_traj[3] = 0.0;

   trajectory_data.y_traj[0] = 0.0;
   trajectory_data.y_traj[1] = 0.0;
   trajectory_data.y_traj[2] = 0.0;
   trajectory_data.y_traj[3] = 0.0;

   trajectory_data.z_traj[0] = 0.5;
   trajectory_data.z_traj[1] = 0.5;
   trajectory_data.z_traj[2] = 0.5;
   trajectory_data.z_traj[3] = 0.5;

   //In radians
   trajectory_data.yaw_traj[0] = 0;
   trajectory_data.yaw_traj[1] = 0;
   trajectory_data.yaw_traj[2] = 0;
   trajectory_data.yaw_traj[3] = 0;

   trajectory_data.radius_traj[0] = 0.01;
   trajectory_data.radius_traj[1] = 0.01;
   trajectory_data.radius_traj[2] = 0.01;
   trajectory_data.radius_traj[3] = 0.01;

   trajectory_data.current_point = 0;
   trajectory_data.completed = false;

   position_data.goal_x = trajectory_data.x_traj[trajectory_data.current_point];
   position_data.goal_y = trajectory_data.y_traj[trajectory_data.current_point];
   position_data.goal_z = trajectory_data.z_traj[trajectory_data.current_point];
   position_data.goal_yaw = trajectory_data.yaw_traj[trajectory_data.current_point];
   position_data.goal_range = trajectory_data.radius_traj[trajectory_data.current_point];

   position_data.pos_x = 0.0;
   position_data.pos_y = 0.0;
   position_data.pos_z = 0.0;
   position_data.pos_yaw = 0.0;

   #if REAL
      real_cmd.cmd[0] = true;
      real_cmd.cmd[1] = true;
      real_cmd.cmd[2] = true;
      real_cmd.cmd[3] = true;
   #endif
}

void AscTecController::set_parameters(struct PARAM_DATA * param_ptr)
{
   k_p_x = param_ptr->k_p_x;
   k_i_x = param_ptr->k_i_x;
   k_d_x = param_ptr->k_d_x;

   k_p_y = param_ptr->k_p_y;
   k_i_y = param_ptr->k_i_y;
   k_d_y = param_ptr->k_d_y;

   k_p_z = param_ptr->k_p_z;
   k_i_z = param_ptr->k_i_z;
   k_d_z = param_ptr->k_d_z;

   k_p_yaw = param_ptr->k_p_yaw;
   k_i_yaw = param_ptr->k_i_yaw;
   k_d_yaw = param_ptr->k_d_yaw;

}

void AscTecController::set_frames(string world_frame, string quad_frame)
{
   global_frame = world_frame;
   asctec_frame = quad_frame;
}

void AscTecController::update_position(float x, float y, float z, float yaw)
{
   position_data.pos_x = x;
   position_data.pos_y = y;
   position_data.pos_z = z;
   position_data.pos_yaw = yaw;
}

bool AscTecController::goal_arrived(void)
{
   float distance = (ctl_data.error_x_cur) * (ctl_data.error_x_cur) + 
                           (ctl_data.error_y_cur) * (ctl_data.error_y_cur) + 
                           (ctl_data.error_z_cur) * (ctl_data.error_z_cur); 
   
   if((distance <= position_data.goal_range) && not (position_data.goal_arrived)) { 
     ROS_INFO("Goal Reached");
     return true;
   }

   return false;
}

bool AscTecController::update_goal(void)
{
   if(trajectory_data.current_point == (TRAJ_SIZE - 1) && not (trajectory_data.completed)) {
      ROS_INFO("Trajectory Completed");
      trajectory_data.completed = true;
      return true;
   }
   trajectory_data.current_point++;

   position_data.goal_x = trajectory_data.x_traj[trajectory_data.current_point];
   position_data.goal_y = trajectory_data.y_traj[trajectory_data.current_point];   
   position_data.goal_z = trajectory_data.z_traj[trajectory_data.current_point];
   position_data.goal_yaw = trajectory_data.yaw_traj[trajectory_data.current_point];
   position_data.goal_range = trajectory_data.radius_traj[trajectory_data.current_point];

   ROS_INFO("New Waypoint Set");
   return false;
}

void AscTecController::update_controller(void)
{
   ctl_data.error_x_past = ctl_data.error_x_cur;
   ctl_data.error_y_past = ctl_data.error_y_cur;
   ctl_data.error_z_past = ctl_data.error_z_cur;
   ctl_data.error_yaw_past = ctl_data.error_yaw_cur;

   /* ---- P calculation ---- */
   ctl_data.error_x_cur = (position_data.goal_x) - (position_data.pos_x);
   ctl_data.error_y_cur = (position_data.goal_y) - (position_data.pos_y);
   ctl_data.error_z_cur = (position_data.goal_z) - (position_data.pos_z);
   ctl_data.error_yaw_cur = (position_data.goal_yaw) - (position_data.pos_yaw);

   /* ---- I calculation ---- */
   ctl_data.integral_x += (ctl_data.error_x_cur) * dt;
   ctl_data.integral_y += (ctl_data.error_y_cur) * dt;
   ctl_data.integral_z += (ctl_data.error_z_cur) * dt;
   ctl_data.integral_yaw += (ctl_data.error_yaw_cur) * dt;

   /*
   if(ctl_data.integral_x > integral_limit) 
   {
      ctl_data.integral_x = integral_limit;
   }

   if(ctl_data.integral_y > integral_limit) 
   {
      ctl_data.integral_y = integral_limit;
   }

   if(ctl_data.integral_z > integral_limit) 
   {
      ctl_data.integral_z = integral_limit;
   }
   */

   /* ---- D calculation ---- */
   ctl_data.diff_x = ((ctl_data.error_x_cur) - 
                            (ctl_data.error_x_past)) / dt;
   ctl_data.diff_y = ((ctl_data.error_y_cur) - 
                            (ctl_data.error_y_past)) / dt;
   ctl_data.diff_z = ((ctl_data.error_z_cur) - 
                            (ctl_data.error_z_past)) / dt;
   ctl_data.diff_yaw = ((ctl_data.error_yaw_cur) - 
                            (ctl_data.error_yaw_past)) / dt;
}

void AscTecController::update_real_cmd(void)
{

   float roll;
   float pitch;
   float thrust;
   float yaw;

   yaw = -(k_p_yaw * (ctl_data.error_yaw_cur) + 
                         k_i_yaw * (ctl_data.integral_yaw) + 
                         k_d_yaw * (ctl_data.diff_yaw));
   
   if(yaw > YAW_MAX) {
      yaw = YAW_MAX;
   }else if(yaw < YAW_MIN) {
      yaw = YAW_MIN;
   }

   real_cmd.yaw = M_PI * (yaw / YAW_MAX);

   pitch = -((k_p_x * (ctl_data.error_x_cur) + 
                         k_i_x * (ctl_data.integral_x) + 
                         k_d_x * 50 * (ctl_data.diff_x)) *
                         cos(position_data.pos_yaw)) + 
           -((k_p_y * (ctl_data.error_y_cur) + 
                          k_i_y * (ctl_data.integral_y) + 
                          k_d_y * 50 * (ctl_data.diff_y)) *
                          sin(position_data.pos_yaw));
   
   if(pitch > PITCH_MAX) {
      pitch = PITCH_MAX;
   }else if(pitch < PITCH_MIN) {
      pitch = PITCH_MIN;
   }

   real_cmd.pitch = M_PI * (pitch / PITCH_MAX) / 2;

   roll = ((k_p_x * (ctl_data.error_x_cur) + 
                         k_i_x * (ctl_data.integral_x) + 
                         k_d_x * 50 * (ctl_data.diff_x)) *
                         sin(position_data.pos_yaw)) + 
           -((k_p_y * (ctl_data.error_y_cur) + 
                          k_i_y * (ctl_data.integral_y) + 
                          k_d_y * 50 * (ctl_data.diff_y)) *
                          cos(position_data.pos_yaw));
   
   if(roll > ROLL_MAX) {
      roll = ROLL_MAX;
   }else if(roll < ROLL_MIN) {
      roll = ROLL_MIN;
   }

   real_cmd.roll = M_PI * (roll / ROLL_MAX) / 2;

   thrust = k_p_z * (ctl_data.error_z_cur) + 
                           k_i_z * (ctl_data.integral_z) + 
                           k_d_z * 50 * (ctl_data.diff_z);

   if(thrust > THRUST_MAX) {
      thrust = THRUST_MAX;
   }else if(thrust < THRUST_MIN) {
      thrust = THRUST_MIN;
   }
   
   real_cmd.thrust = thrust / THRUST_MAX;
}

void AscTecController::update_sim_cmd(void)
{   
   sim_cmd.yaw = 0.0;
   sim_cmd.acceleration.x = k_p_x * (ctl_data.error_x_cur) + 
                             k_i_x * (ctl_data.integral_x) + 
                             k_d_x * (ctl_data.diff_x);

   sim_cmd.acceleration.y = k_p_y * (ctl_data.error_y_cur) + 
                             k_i_y * (ctl_data.integral_y) + 
                             k_d_y * (ctl_data.diff_y);

   sim_cmd.acceleration.z = k_p_z * (ctl_data.error_z_cur) + 
                             k_i_z * (ctl_data.integral_z) + 
                             k_d_z * (ctl_data.diff_z);
}
