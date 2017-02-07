#include <controller.h>
	
pc_asctec_sim::SICmd real_cmd;
pc_asctec_sim::pc_feedback on_goal;
pc_asctec_sim::pc_state state_data;

PID_DATA ctl_data;
POS_DATA position_data;

ros::Time past, now;

double dt = 0.0;

float battery = 0.0;
int battery_status = 4;

float k_p_x, k_i_x, k_d_x, k_a_x;
float k_p_y, k_i_y, k_d_y, k_a_y;
float k_p_z, k_i_z, k_d_z, k_a_z;
float k_p_yaw, k_i_yaw, k_d_yaw, k_a_yaw;

bool steady = false;
bool counting = false;

float final_x, final_y, final_z, final_yaw;
int yaw_counter = 0;

ros::Publisher goal_feedback, accel_quad_cmd, state_pub, trail_pub;
ros::Subscriber pc_goal_cmd, joy_call, bat_sub, start_sub;
visualization_msgs::Marker trail;

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
   pos_ptr->goal_z = 0.0;
   pos_ptr->goal_yaw = 0.0;

   pos_ptr->goal_vel_x = 0.0;
   pos_ptr->goal_vel_y = 0.0;
   pos_ptr->goal_vel_z = 0.0;
   pos_ptr->goal_vel_yaw = 0.0;

   pos_ptr->acc_x_now = 0;
   pos_ptr->acc_y_now = 0;
   pos_ptr->acc_z_now = 0;
   pos_ptr->acc_yaw_now = 0;

   for(int i = 0; i < BUFFER; i++) {
      pos_ptr->acc_x_buf[i] = 0;
      pos_ptr->acc_y_buf[i] = 0;
      pos_ptr->acc_z_buf[i] = 0;
      pos_ptr->acc_yaw_buf[i] = 0;
   }

   pos_ptr->goal_range = 0.05;
   pos_ptr->wait_time = 0.0;
   pos_ptr->waiting = false;
   pos_ptr->goal_arrival = false;
   pos_ptr->goal_id = "Init";

   real_cmd.cmd[0] = true;
   real_cmd.cmd[1] = true;
   real_cmd.cmd[2] = true;
   real_cmd.cmd[3] = true;
}

bool setBatteryStatus(void) 
{
   bool isGood = true;
   ROS_INFO("Starting Battery: %f V", battery);
   if(battery >= BATTERY_FULL) {
      battery_status = 4;
   }else if(battery >= BATTERY_MID) {
      battery_status = 3;
   }else if(battery > BATTERY_EMPTY) {
      battery_status = 2;
   }else if(battery <= BATTERY_EMPTY) {
      ROS_INFO("Cannot Fly, Battery is too low!!");
      isGood = false;
   }
   return isGood;
}

bool goal_arrived(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr)
{
   float distance = sqrt((ctl_ptr->error_x) * (ctl_ptr->error_x) + 
                           (ctl_ptr->error_y) * (ctl_ptr->error_y) + 
                           (ctl_ptr->error_z) * (ctl_ptr->error_z)); 
   
   if((distance <= (pos_ptr->goal_range)) && not (pos_ptr->goal_arrival)) {
      pos_ptr->wait_start = ros::Time::now().toSec(); 
      pos_ptr->waiting = true;  
      pos_ptr->goal_arrival = true;
   }
   if(pos_ptr->waiting) {
      if((ros::Time::now().toSec() - pos_ptr->wait_start) >= pos_ptr->wait_time) {
         on_goal.goal_id = pos_ptr->goal_id;
         on_goal.event = ros::Time::now();
         goal_feedback.publish(on_goal);
         pos_ptr->waiting = false;
         return true;
      }
   }
   return false;
}

void check_bat(void) {
	
	if(battery <= BATTERY_MID && battery_status == 3) {
		ROS_INFO("50 Percent Battery Remaining: %f V", battery);
		battery_status = 2;	
	}else if(battery <= BATTERY_EMPTY && battery_status == 2) {
		ROS_INFO("10 Percent Battery Remaining: %f V, land now!!", battery);
		battery_status = 1;
	}
}

void callback(const pc_asctec_sim::tunerConfig &config, uint32_t level) {

	k_p_x = config.k_p_x;
	k_i_x = config.k_i_x;
	k_d_x = config.k_d_x;
	k_a_x = config.k_a_x;

	k_p_y = config.k_p_y;
	k_i_y = config.k_i_y;
	k_d_y = config.k_d_y;
	k_a_y = config.k_a_y;

	k_p_z = config.k_p_z;
	k_i_z = config.k_i_z;
	k_d_z = config.k_d_z;
	k_a_z = config.k_a_z;

	k_p_yaw = config.k_p_yaw;
	k_i_yaw = config.k_i_yaw;
	k_d_yaw = config.k_d_yaw;
	k_a_yaw = config.k_a_yaw;
}

void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{
       position_data.goal_x = msg->x;
       position_data.goal_y = msg->y;
       position_data.goal_z = msg->z;
       position_data.goal_yaw = msg->yaw;

       position_data.goal_vel_x = msg->x_vel;
       position_data.goal_vel_y = msg->y_vel;
       position_data.goal_vel_z = msg->z_vel;
       position_data.goal_vel_yaw = msg->yaw_vel;

       position_data.goal_acc_x = msg->x_acc;
       position_data.goal_acc_y = msg->y_acc;
       position_data.goal_acc_z = msg->z_acc;
       position_data.goal_acc_yaw = msg->yaw_acc;

       position_data.goal_arrival = false;
       position_data.waiting = false;

       position_data.wait_time = msg->wait_time;
       position_data.goal_range = msg->goal_limit;
       position_data.goal_id = msg->goal_id;  
}

void ll_callback(const pc_asctec_sim::LLStatus::ConstPtr& msg)
{
   battery = msg->battery_voltage;
}

float limit(float input, float ceiling, float floor) 
{
   if(input > ceiling) {
      return ceiling;
   }else if(input < floor) {
      return floor;
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

   /* ---- Vel calculation ---- */
   controller_ptr->error_x_vel = (position_ptr->goal_vel_x) - 
                                 (position_ptr->vel_x);
   controller_ptr->error_y_vel = (position_ptr->goal_vel_y) - 
                                 (position_ptr->vel_y);
   controller_ptr->error_z_vel = (position_ptr->goal_vel_z) - 
                                 (position_ptr->vel_z); 
   controller_ptr->error_yaw_vel = (position_ptr->goal_vel_yaw) - 
                                   (position_ptr->vel_yaw);

   /* ---- Acc calculation ---- */
   controller_ptr->error_x_acc = (position_ptr->goal_acc_x) - 
                                 (position_ptr->acc_x);
   controller_ptr->error_y_acc = (position_ptr->goal_acc_y) - 
                                 (position_ptr->acc_y);
   controller_ptr->error_z_acc = (position_ptr->goal_acc_z) - 
                                 (position_ptr->acc_z); 
   controller_ptr->error_yaw_acc = (position_ptr->goal_acc_yaw) - 
                                   (position_ptr->acc_yaw);

   /* ---- Windup Prevention ---- */
   controller_ptr->integral_x = limit(controller_ptr->integral_x, INTEGRAL_LIMIT, 0);
   controller_ptr->integral_y = limit(controller_ptr->integral_y, INTEGRAL_LIMIT, 0);
   controller_ptr->integral_z = limit(controller_ptr->integral_z, INTEGRAL_LIMIT, 0);
   controller_ptr->integral_yaw = limit(controller_ptr->integral_yaw, INTEGRAL_LIMIT, 0);
}

void update_real_cmd(struct PID_DATA * ctl_ptr, struct POS_DATA * pos_ptr)
{
   double roll;
   double pitch;
   double thrust;
   double yaw;

   yaw = -(k_p_yaw * (ctl_ptr->error_yaw) + 
           k_i_yaw * (ctl_ptr->integral_yaw) + 
           k_d_yaw * (ctl_ptr->error_yaw_vel) +
           k_a_yaw * (ctl_ptr->error_yaw_acc));

   yaw = limit(yaw, YAW_MAX, YAW_MIN);
   
   if(!isnan(M_PI * (yaw / YAW_MAX))) {
      real_cmd.yaw = M_PI * (yaw / YAW_MAX);
   }

   pitch = -((k_p_x * (ctl_ptr->error_x) + 
              k_i_x * (ctl_ptr->integral_x) + 
              k_d_x * (ctl_ptr->error_x_vel) + 
	      k_a_x * (ctl_ptr->error_x_acc)) *
              cos(pos_ptr->pos_yaw)) + 

           -((k_p_y * (ctl_ptr->error_y) + 
              k_i_y * (ctl_ptr->integral_y) + 
              k_d_y * (ctl_ptr->error_y_vel) + 
	      k_a_y * (ctl_ptr->error_y_acc)) *
              sin(pos_ptr->pos_yaw)); 
   
   pitch = limit(pitch, PITCH_MAX, PITCH_MIN);

   if(!isnan(M_PI * (pitch / PITCH_MAX) / 24)) {
      real_cmd.pitch = M_PI * (pitch / PITCH_MAX) / 24;
   }

   roll = ((k_p_x * (ctl_ptr->error_x) + 
            k_i_x * (ctl_ptr->integral_x) + 
            k_d_x * (ctl_ptr->error_x_vel) +
	    k_a_x * (ctl_ptr->error_x_acc)) *
            sin(pos_ptr->pos_yaw)) +

          -((k_p_y * (ctl_ptr->error_y) + 
             k_i_y * (ctl_ptr->integral_y) + 
             k_d_y * (ctl_ptr->error_y_vel) + 
	     k_a_y * (ctl_ptr->error_y_acc)) *
             cos(pos_ptr->pos_yaw));

   roll = limit(roll, ROLL_MAX, ROLL_MIN);

   if(!isnan(M_PI * (roll / ROLL_MAX) / 24)) {
      real_cmd.roll = M_PI * (roll / ROLL_MAX) / 24;
   }

   thrust = k_p_z * (ctl_ptr->error_z) + 
            k_i_z * (ctl_ptr->integral_z) + 
            k_d_z * (ctl_ptr->error_z_vel) +
	    k_a_z * (ctl_ptr->error_z_acc) + G_TH;

   thrust = limit(thrust, THRUST_MAX, THRUST_MIN);

   if(!isnan(thrust / THRUST_MAX)) {
      real_cmd.thrust = thrust / THRUST_MAX;
   }
}

void setAvgAccel(struct POS_DATA * pos_ptr)
{
   float tempx, tempy, tempz, tempyaw;
   tempx = tempy = tempz = tempyaw = 0.0;

   for(int i = 0; i < BUFFER; i++) {
      tempx += pos_ptr->acc_x_buf[i];
      tempy += pos_ptr->acc_y_buf[i];
      tempz += pos_ptr->acc_z_buf[i];
      tempyaw += pos_ptr->acc_yaw_buf[i];
   }

   pos_ptr->acc_x = tempx / BUFFER;
   pos_ptr->acc_y = tempy / BUFFER;
   pos_ptr->acc_z = tempz / BUFFER;
   pos_ptr->acc_yaw = tempyaw / BUFFER;
}

void publish_trail(float x, float y, float z)
{
	geometry_msgs::Point vis_trail;
	vis_trail.x = x;
	vis_trail.y = y;
	vis_trail.z = z;

	trail.points.push_back(vis_trail);
	trail_pub.publish(trail);
	trail.points.push_back(vis_trail);
}

void init_trail(string str_frame)
{
	trail.header.frame_id = str_frame;
	trail.header.stamp = ros::Time::now();
	trail.id = 2;
	trail.action = visualization_msgs::Marker::ADD;
	trail.type = visualization_msgs::Marker::LINE_LIST;
	trail.color.a = 1.0;				
	trail.color.b = 1.0;
	trail.color.g = 0.7;

	trail.scale.x = 0.05;
	trail.scale.y = 0.05;

	geometry_msgs::Point vis_trail;
	vis_trail.x = position_data.pos_x;
	vis_trail.y = position_data.pos_y;
	vis_trail.z = position_data.pos_z;
	trail.points.push_back(vis_trail);
}

int main(int argc, char** argv) {
   
   ros::init(argc, argv, "pos_controller");
   ros::NodeHandle nh;

   dynamic_reconfigure::Server<pc_asctec_sim::tunerConfig> server;
   dynamic_reconfigure::Server<pc_asctec_sim::tunerConfig>::CallbackType f;
   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);

   ROS_INFO("Dynamic Reconfigure Server Started");

   string name, world_frame, frame;
   ros::param::get("~world_frame", world_frame);
   ros::param::get("~name", name);
   ros::param::get("~quad_frame", frame);

   ros::param::get("~k_p_x", k_p_x); 
   ros::param::get("~k_i_x", k_i_x); 
   ros::param::get("~k_d_x", k_d_x);
   ros::param::get("~k_a_x", k_a_x);

   ros::param::get("~k_p_y", k_p_y); 
   ros::param::get("~k_i_y", k_i_y); 
   ros::param::get("~k_d_y", k_d_y);
   ros::param::get("~k_a_y", k_a_y);

   ros::param::get("~k_p_z", k_p_z); 
   ros::param::get("~k_i_z", k_i_z); 
   ros::param::get("~k_d_z", k_d_z);   
   ros::param::get("~k_a_z", k_a_z);
 
   ros::param::get("~k_p_yaw", k_p_yaw); 
   ros::param::get("~k_i_yaw", k_i_yaw); 
   ros::param::get("~k_d_yaw", k_d_yaw);
   ros::param::get("~k_a_yaw", k_a_yaw);

   accel_quad_cmd = nh.advertise<pc_asctec_sim::SICmd>(name + "/cmd_si", 10);
   goal_feedback = nh.advertise<pc_asctec_sim::pc_feedback>(name + "/goal_feedback", 10);
   state_pub = nh.advertise<pc_asctec_sim::pc_state>(name + "/state", 10);
   trail_pub = nh.advertise<visualization_msgs::Marker>(name + "/quad_trail",10);

   pc_goal_cmd = nh.subscribe(name + "/pos_goals", 1, goal_callback);
   bat_sub = nh.subscribe(name + "/ll_status", 1, ll_callback);
   ros::Rate rate(CONTROL_RATE);

   tf::StampedTransform transform;
   tf::TransformListener listener;

   PID_DATA * controller_ptr = &ctl_data;
   POS_DATA * position_ptr = &position_data;

   init(position_ptr, controller_ptr);
   init_trail(world_frame);
   ros::Duration(1.5).sleep();
   listener.waitForTransform(world_frame, frame, 
                             ros::Time(0), ros::Duration(3.0));
   ros::spinOnce();
   if(!setBatteryStatus()) {
      ros::spin();
   }

   listener.lookupTransform(world_frame, frame, ros::Time(0), transform);
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
   
   now = transform.stamp_;
   past = now;

   double timer_past, timer = 0;
   bool check = false;
   ROS_INFO("Controller active!!");

   while (ros::ok()) {
    //Check Battery Life
      check_bat();
    
    //Grab new transform data
      listener.lookupTransform(world_frame, frame, ros::Time(0), transform);
      now = transform.stamp_;
      //dt = (now.toSec() - past.toSec()) + (now.toNSec() - past.toNSec())/pow(10,9);
	dt = (now.toSec() - past.toSec());

    //Set t-1 values
      position_ptr->pos_x_past = position_ptr->pos_x;
      position_ptr->pos_y_past = position_ptr->pos_y;
      position_ptr->pos_z_past = position_ptr->pos_z;
      position_ptr->pos_yaw_past = position_ptr->pos_yaw;

      position_ptr->vel_x_past = position_ptr->vel_x;
      position_ptr->vel_y_past = position_ptr->vel_y;
      position_ptr->vel_z_past = position_ptr->vel_z;
      position_ptr->vel_yaw_past = position_ptr->vel_yaw;
      
      past = now;

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
      position_ptr->vel_x = ((position_ptr->pos_x) - (position_ptr->pos_x_past)) / dt;
      position_ptr->vel_y = ((position_ptr->pos_y) - (position_ptr->pos_y_past)) / dt;
      position_ptr->vel_z = ((position_ptr->pos_z) - (position_ptr->pos_z_past)) / dt;
      position_ptr->vel_yaw = ((position_ptr->pos_yaw) - (position_ptr->pos_yaw_past)) / dt;

    //Calculate accel values
      position_ptr->acc_x_buf[position_ptr->acc_x_now] = ((position_ptr->vel_x) - (position_ptr->vel_x_past)) / dt;
      position_ptr->acc_y_buf[position_ptr->acc_y_now] = ((position_ptr->vel_y) - (position_ptr->vel_y_past)) / dt;
      position_ptr->acc_z_buf[position_ptr->acc_z_now] = ((position_ptr->vel_z) - (position_ptr->vel_z_past)) / dt;
      position_ptr->acc_yaw_buf[position_ptr->acc_yaw_now] = ((position_ptr->vel_yaw) - (position_ptr->vel_yaw_past)) / dt;

      position_ptr->acc_x_now++;
      position_ptr->acc_y_now++;
      position_ptr->acc_z_now++;
      position_ptr->acc_yaw_now++;

      position_ptr->acc_x_now &= BUFFER-1;
      position_ptr->acc_y_now &= BUFFER-1;
      position_ptr->acc_z_now &= BUFFER-1;
      position_ptr->acc_yaw_now &= BUFFER-1;   

      setAvgAccel(position_ptr);

    //Update controller values
      update_controller(controller_ptr, position_ptr);   

    //Check if target goal was accomplished -> update new goal and publish goal confirm  
      goal_arrived(controller_ptr, position_ptr);

      update_real_cmd(controller_ptr, position_ptr);
      accel_quad_cmd.publish(real_cmd);

    //Publish trail
      publish_trail(position_ptr->pos_x, position_ptr->pos_y, position_ptr->pos_z);

    //Fill State Data and Publish
      state_data.event = now;

      state_data.x = position_ptr->pos_x;
      state_data.x_vel = position_ptr->vel_x;
      state_data.x_acc = position_ptr->acc_x;
      state_data.x_goal = position_ptr->goal_x;
      state_data.x_vel_goal = position_ptr->goal_vel_x;
      state_data.x_acc_goal = position_ptr->goal_acc_x;

      state_data.y = position_ptr->pos_y;
      state_data.y_vel = position_ptr->vel_y;
      state_data.y_acc = position_ptr->acc_y;
      state_data.y_goal = position_ptr->goal_y;
      state_data.y_vel_goal = position_ptr->goal_vel_y;
      state_data.y_acc_goal = position_ptr->goal_acc_y;

      state_data.z = position_ptr->pos_z;
      state_data.z_vel = position_ptr->vel_z;
      state_data.z_acc = position_ptr->acc_z;
      state_data.z_goal = position_ptr->goal_z;
      state_data.z_vel_goal = position_ptr->goal_vel_z;
      state_data.z_acc_goal = position_ptr->goal_acc_z;

      state_data.yaw = position_ptr->pos_yaw;
      state_data.yaw_vel = position_ptr->vel_yaw;
      state_data.yaw_acc = position_ptr->acc_yaw;
      state_data.yaw_goal = position_ptr->goal_yaw;
      state_data.yaw_vel_goal = position_ptr->goal_vel_yaw;
      state_data.yaw_acc_goal = position_ptr->goal_acc_yaw;

      state_pub.publish(state_data);

      ros::spinOnce();
      rate.sleep();
   }
   return 0;
}
