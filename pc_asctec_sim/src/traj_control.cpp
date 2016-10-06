#include <traj_control.h>

quadrotor_msgs::PositionCommand sim_cmd;	
pc_asctec_sim::SICmd real_cmd;
pc_asctec_sim::pc_feedback on_goal;

PID_DATA ctl_data;
POS_DATA position_data;

float k_p_xy;
float k_i_xy;
float k_d_xy;

float k_p_z;
float k_i_z;
float k_d_z;

float k_p_yaw;
float k_i_yaw;
float k_d_yaw;

bool halt = false;
bool centered = false;

float final_x, final_y, final_z, final_yaw;
int yaw_counter = 0;
string name;

ros::Publisher goal_feedback;
ros::Publisher accel_quad_cmd;
ros::Subscriber pc_goal_cmd;
ros::Subscriber shutdown_request;
ros::Subscriber center_shutdown_request;

void init(struct POS_DATA * pos_ptr, struct PID_DATA * ctl_ptr)
{
   //initialize controller
   ctl_ptr->error_x = 0.0;
   ctl_ptr->integral_x = 0.0;
   ctl_ptr->error_x_vel = 0.0;

   ctl_ptr->error_y = 0.0;
   ctl_ptr->integral_y = 0.0;
   ctl_ptr->error_y_vel = 0.0;

   ctl_ptr->error_z = 0.0;
   ctl_ptr->integral_z = 0.0;
   ctl_ptr->error_z_vel = 0.0;

   ctl_ptr->error_yaw = 0.0;
   ctl_ptr->integral_yaw = 0.0;
   ctl_ptr->error_yaw_vel = 0.0;
   
   //initialize position data
   pos_ptr->pos_x = 0.0;
   pos_ptr->pos_y = 0.0;
   pos_ptr->pos_z = 0.0;
   pos_ptr->pos_yaw = 0.0;

   pos_ptr->pos_x_past = 0.0;
   pos_ptr->pos_y_past = 0.0;
   pos_ptr->pos_z_past = 0.0;
   pos_ptr->pos_yaw_past = 0.0;

   pos_ptr->vel_x = 0.0;
   pos_ptr->vel_y = 0.0;
   pos_ptr->vel_z = 0.0;
   pos_ptr->vel_yaw = 0.0;

   pos_ptr->goal_x = 0.0;
   pos_ptr->goal_y = 0.0;
   pos_ptr->goal_z = 1.0;
   pos_ptr->goal_yaw = 0.0;

   pos_ptr->goal_vel_x = 0.0;
   pos_ptr->goal_vel_y = 0.0;
   pos_ptr->goal_vel_z = 0.0;
   pos_ptr->goal_vel_yaw = 0.0;

   pos_ptr->goal_range = 0.05;
   pos_ptr->goal_arrival = false;
   pos_ptr->goal_id = "Init";

   real_cmd.cmd[0] = true;
   real_cmd.cmd[1] = true;
   real_cmd.cmd[2] = true;
   real_cmd.cmd[3] = true;
}

bool goal_arrived(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr)
{
   float distance = (ctl_ptr->error_x) * (ctl_ptr->error_x) + 
                           (ctl_ptr->error_y) * (ctl_ptr->error_y) + 
                           (ctl_ptr->error_z) * (ctl_ptr->error_z); 
   
   if((distance <= (pos_ptr->goal_range)) && not (pos_ptr->goal_arrival)) { 
     pos_ptr->goal_arrival = true;
     on_goal.goal_id = pos_ptr->goal_id;
     on_goal.event = ros::Time::now();
     goal_feedback.publish(on_goal);
     if(pos_ptr->goal_id == "center") {
        centered = true;
	ROS_INFO("Centering behavior completed, now landing...");
     }
     return true;
   }
   return false;
}

void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{
   if((position_data.goal_id != msg->goal_id) && !halt) {
       position_data.goal_x = msg->x;
       position_data.goal_y = msg->y;
       position_data.goal_z = msg->z;
       position_data.goal_yaw = msg->yaw;
       position_data.goal_vel_x = msg->x_vel;
       position_data.goal_vel_y = msg->y_vel;
       position_data.goal_vel_z = msg->z_vel;
       position_data.goal_vel_yaw = msg->yaw_vel;

       position_data.goal_arrival = false;
       position_data.goal_range = msg->goal_limit;
       position_data.goal_id = msg->goal_id;      
    }
}

void center_shutdown_callback(const std_msgs::Empty::ConstPtr& msg)
{
   ROS_INFO_STREAM("Center Landing Quadrotor " + name);
   position_data.goal_x = 0.0;
   position_data.goal_y = 0.0;
   position_data.goal_z = 1.0;
   position_data.goal_yaw = 0.0 + 2*M_PI*yaw_counter;
   position_data.goal_range = 0.01;
   position_data.goal_arrival = false;
   position_data.goal_id = "center";
   halt = true;
}

void shutdown_callback(const std_msgs::Empty::ConstPtr& msg)
{
   ROS_INFO_STREAM("Landing Quadrotor " + name);
   position_data.goal_x = position_data.pos_x;
   position_data.goal_y = position_data.pos_y;
   position_data.goal_yaw = position_data.pos_yaw;
   halt = true;
}

void land_cmd(struct POS_DATA * position_ptr) {

   if(position_ptr->goal_z > 0.5 && position_ptr->goal_arrival) {
      position_ptr->goal_z -= 0.02;
      position_ptr->goal_range = 0.001;
   }
}

float limit(float input, float ceiling) 
{
   if(input > ceiling) {
      return ceiling;
   }else {
      return input;
   }
}

void update_controller(struct PID_DATA * controller_ptr, struct POS_DATA * position_ptr)
{
   /* ---- P calculation ---- */
   controller_ptr->error_x = (position_ptr->goal_x) - (position_ptr->pos_x);
   controller_ptr->error_y = (position_ptr->goal_y) - (position_ptr->pos_y);
   controller_ptr->error_z = (position_ptr->goal_z) - (position_ptr->pos_z);
   controller_ptr->error_yaw = (position_ptr->goal_yaw) - (position_ptr->pos_yaw);

   /* ---- I calculation ---- */
   controller_ptr->integral_x += (controller_ptr->error_x) * dt;
   controller_ptr->integral_y += (controller_ptr->error_y) * dt;
   controller_ptr->integral_z += (controller_ptr->error_z) * dt;
   controller_ptr->integral_yaw += (controller_ptr->error_yaw) * dt;

   /* ---- D calculation ---- */
   controller_ptr->error_x_vel = (position_ptr->goal_vel_x) - 
                                 (position_ptr->vel_x);
   controller_ptr->error_y_vel = (position_ptr->goal_vel_y) - 
                                 (position_ptr->vel_y);
   controller_ptr->error_z_vel = (position_ptr->goal_vel_z) - 
                                 (position_ptr->vel_z); 
   controller_ptr->error_yaw_vel = (position_ptr->goal_vel_yaw) - 
                                   (position_ptr->vel_yaw);

   /* ---- Windup Prevention ---- */
   controller_ptr->integral_x = limit(controller_ptr->integral_x, INTEGRAL_LIMIT);
   controller_ptr->integral_y = limit(controller_ptr->integral_y, INTEGRAL_LIMIT);
   controller_ptr->integral_z = limit(controller_ptr->integral_z, INTEGRAL_LIMIT);
   controller_ptr->integral_yaw = limit(controller_ptr->integral_yaw, INTEGRAL_LIMIT);
}

void update_real_cmd(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr)
{
   float roll;
   float pitch;
   float thrust;
   float yaw;

   yaw = -(k_p_yaw * (ctl_ptr->error_yaw) + 
                         k_i_yaw * (ctl_ptr->integral_yaw) + 
                         k_d_yaw * (ctl_ptr->error_yaw_vel));
   
   if(yaw > YAW_MAX) {
      yaw = YAW_MAX;
   }else if(yaw < YAW_MIN) {
      yaw = YAW_MIN;
   }

   real_cmd.yaw = M_PI * (yaw / YAW_MAX);

   pitch = -((k_p_xy * (ctl_ptr->error_x) + 
                         k_i_xy * (ctl_ptr->integral_x) + 
                         k_d_xy * (ctl_ptr->error_x_vel)) *
                         cos(pos_ptr->pos_yaw)) + 
           -((k_p_xy * (ctl_ptr->error_y) + 
                          k_i_xy * (ctl_ptr->integral_y) + 
                          k_d_xy * (ctl_ptr->error_y_vel)) *
                          sin(pos_ptr->pos_yaw)); 
   
   if(pitch > PITCH_MAX) {
      pitch = PITCH_MAX;
   }else if(pitch < PITCH_MIN) {
      pitch = PITCH_MIN;
   }

   real_cmd.pitch = M_PI * (pitch / PITCH_MAX) / 12;

   roll = ((k_p_xy * (ctl_ptr->error_x) + 
                         k_i_xy * (ctl_ptr->integral_x) + 
                         k_d_xy * (ctl_ptr->error_x_vel)) *
                         sin(pos_ptr->pos_yaw)) +
          -((k_p_xy * (ctl_ptr->error_y) + 
                          k_i_xy * (ctl_ptr->integral_y) + 
                          k_d_xy * (ctl_ptr->error_y_vel)) *
                          cos(pos_ptr->pos_yaw));
   
   if(roll > ROLL_MAX) {
      roll = ROLL_MAX;
   }else if(roll < ROLL_MIN) {
      roll = ROLL_MIN;
   }

   real_cmd.roll = M_PI * (roll / ROLL_MAX) / 12;

   thrust = k_p_z * (ctl_ptr->error_z) + 
                           k_i_z * (ctl_ptr->integral_z) + 
                           k_d_z * (ctl_ptr->error_z_vel);

   if(thrust > THRUST_MAX) {
      thrust = THRUST_MAX;
   }else if(thrust < THRUST_MIN) {
      thrust = THRUST_MIN;
   }
   
   real_cmd.thrust = thrust / THRUST_MAX;
}

int main(int argc, char** argv) {
   
   ros::init(argc, argv, "pos_controller");
   ros::NodeHandle nh;

   string world_frame, quad_frame, name;
   ros::param::get("~world_frame", world_frame);
   ros::param::get("~quad_frame", quad_frame);
   ros::param::get("~quad_name", name);

   ros::param::get("~k_p_xy", k_p_xy); 
   ros::param::get("~k_i_xy", k_i_xy); 
   ros::param::get("~k_d_xy", k_d_xy);

   ros::param::get("~k_p_z", k_p_z); 
   ros::param::get("~k_i_z", k_i_z); 
   ros::param::get("~k_d_z", k_d_z);   
 
   ros::param::get("~k_p_yaw", k_p_yaw); 
   ros::param::get("~k_i_yaw", k_i_yaw); 
   ros::param::get("~k_d_yaw", k_d_yaw);

   accel_quad_cmd = nh.advertise<pc_asctec_sim::SICmd>(name + "/cmd_si", 10);
   goal_feedback = nh.advertise<pc_asctec_sim::pc_feedback>(name + "/goal_feedback", 10);
   pc_goal_cmd = nh.subscribe(name + "/pos_goals", 1, goal_callback);
   shutdown_request = nh.subscribe(name + "/shutdown", 1, shutdown_callback);
   center_shutdown_request = nh.subscribe(name + "/shutdown_center", 1, center_shutdown_callback);

   ros::Rate rate(CONTROL_RATE);

   tf::StampedTransform transform;
   tf::TransformListener listener;

   PID_DATA * controller_ptr = &ctl_data;
   POS_DATA * position_ptr = &position_data;

   init(position_ptr, controller_ptr);
   listener.waitForTransform(world_frame, quad_frame, 
                             ros::Time(0), ros::Duration(10.0));
   ROS_INFO("Transform found!");

   listener.lookupTransform(world_frame, quad_frame, ros::Time(0), transform);
   position_ptr->pos_x = transform.getOrigin().x();
   position_ptr->pos_y = transform.getOrigin().y();
   position_ptr->pos_z = transform.getOrigin().z();  
   position_ptr->pos_yaw = tf::getYaw(transform.getRotation());

   position_ptr->pos_x_past = transform.getOrigin().x();
   position_ptr->pos_y_past = transform.getOrigin().y();
   position_ptr->pos_z_past = transform.getOrigin().z(); 
   position_ptr->pos_yaw_past = tf::getYaw(transform.getRotation());

   position_ptr->goal_x = transform.getOrigin().x();
   position_ptr->goal_y = transform.getOrigin().y();

   ros::Duration(3.0).sleep();   
   double timer_past, timer = 0;
   bool check = false;

   while (ros::ok()) {
    //Grab new transform data
      listener.lookupTransform(world_frame, quad_frame, ros::Time(0), transform);
      
    //Set t-1 position values
      position_ptr->pos_x_past = position_ptr->pos_x;
      position_ptr->pos_y_past = position_ptr->pos_y;
      position_ptr->pos_z_past = position_ptr->pos_z;
      position_ptr->pos_yaw_past = position_ptr->pos_yaw;

    //Get new position values
      position_ptr->pos_x = transform.getOrigin().x();
      position_ptr->pos_y = transform.getOrigin().y();
      position_ptr->pos_z = transform.getOrigin().z();  
      position_ptr->pos_yaw = tf::getYaw(transform.getRotation()) + 2*M_PI*yaw_counter;

    //Adjust yaw counter
      float yaw_dif = position_ptr->pos_yaw - position_ptr->pos_yaw_past;
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

    //Calculate velocity values
      position_ptr->vel_x = (((position_ptr->pos_x) - (position_ptr->pos_x_past)) / dt) * 1000;
      position_ptr->vel_y = (((position_ptr->pos_y) - (position_ptr->pos_y_past)) / dt) * 1000;
      position_ptr->vel_z = (((position_ptr->pos_z) - (position_ptr->pos_z_past)) / dt) * 1000;
      position_ptr->vel_yaw = (((position_ptr->pos_yaw) - (position_ptr->pos_yaw_past)) / dt) * 1000;

    //Update controller values
      update_controller(controller_ptr, position_ptr);
    
    //Check if target goal was accomplished -> update new goal and publish goal confirm  
      goal_arrived(controller_ptr, position_ptr);

      update_real_cmd(controller_ptr, position_ptr);
      accel_quad_cmd.publish(real_cmd);

      if(halt && centered) {
         land_cmd(position_ptr);
      }

      ros::spinOnce();
      rate.sleep();
   }
   return 0;
}
