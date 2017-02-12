#include <controller.h>
#include <tf/transform_broadcaster.h>

pc_asctec_sim::SICmd real_cmd;
pc_asctec_sim::pc_feedback on_goal;
pc_asctec_sim::pc_state state;

CTL_DATA ctl_data;
STATE_DATA pos_data;

ros::Time past, now;

float battery = 0.0;
int battery_status = 4;

float z_offset = 0;

float k_p_x, k_i_x, k_d_x, k_a_x;
float k_p_y, k_i_y, k_d_y, k_a_y;
float k_p_z, k_i_z, k_d_z, k_a_z;
float k_p_yaw, k_i_yaw, k_d_yaw, k_a_yaw;

bool running = false;
bool steady = false;
bool counting = false;

float final_x, final_y, final_z, final_yaw;

ros::Publisher goal_feedback, accel_quad_cmd, state_pub, trail_pub;
ros::Subscriber pc_goal_cmd, joy_call, bat_sub, start_sub;
visualization_msgs::Marker trail;

void init(struct STATE_DATA * pos_ptr, struct CTL_DATA * ctl_ptr)
{
//initialize controller
	ctl_ptr->e_x = 0.0;
	ctl_ptr->i_x = 0.0;
	ctl_ptr->e_vx = 0.0;

	ctl_ptr->e_y = 0.0;
	ctl_ptr->i_y = 0.0;
	ctl_ptr->e_vy = 0.0;

	ctl_ptr->e_z = 0.0;
	ctl_ptr->i_z = 0.0;
	ctl_ptr->e_vz = 0.0;

	ctl_ptr->e_yaw = 0.0;
	ctl_ptr->i_yaw = 0.0;
	ctl_ptr->e_vyaw = 0.0;
	
	//initialize position data
	pos_ptr->x = 0.0;
	pos_ptr->y = 0.0;
	pos_ptr->z = 0.0;
	pos_ptr->yaw = 0.0;

	pos_ptr->x_p = 0.0;
	pos_ptr->y_p = 0.0;
	pos_ptr->z_p = 0.0;
	pos_ptr->yaw_p = 0.0;

	pos_ptr->vx = 0.0;
	pos_ptr->vy = 0.0;
	pos_ptr->vz = 0.0;
	pos_ptr->vyaw = 0.0;

	pos_ptr->g_x = 0.0;
	pos_ptr->g_y = 0.0;
	pos_ptr->g_z = 0.0;
	pos_ptr->g_yaw = 0.0;

	pos_ptr->g_vx = 0.0;
	pos_ptr->g_vy = 0.0;
	pos_ptr->g_vz = 0.0;
	pos_ptr->g_vyaw = 0.0;

	pos_ptr->ax_n = 0;
	pos_ptr->ay_n = 0;
	pos_ptr->az_n = 0;
	pos_ptr->ayaw_n = 0;

	for(int i = 0; i < BUFFER; i++) {
		pos_ptr->ax_buf[i] = 0;
		pos_ptr->ay_buf[i] = 0;
		pos_ptr->az_buf[i] = 0;
		pos_ptr->ayaw_buf[i] = 0;
	}

	pos_ptr->g_range = 0.05;
	pos_ptr->wait_time = 0.0;
	pos_ptr->waiting = false;
	pos_ptr->g_arrival = false;
	pos_ptr->g_id = "Init";

	real_cmd.cmd[0] = true;
	real_cmd.cmd[1] = true;
	real_cmd.cmd[2] = true;
	real_cmd.cmd[3] = true;
}

bool setVStatus(void) 
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

bool checkGoal(struct CTL_DATA * ctl_ptr, struct STATE_DATA * pos_ptr)
{
	float distance = sqrt((ctl_ptr->e_x) * (ctl_ptr->e_x) + (ctl_ptr->e_y) * (ctl_ptr->e_y) + (ctl_ptr->e_z) * (ctl_ptr->e_z)); 
	
	if((distance <= (pos_ptr->g_range)) && not (pos_ptr->g_arrival)) {
		pos_ptr->wait_start = ros::Time::now().toSec(); 
		pos_ptr->waiting = true;  
		pos_ptr->g_arrival = true;
	}
	if(pos_ptr->waiting) {
		if((ros::Time::now().toSec() - pos_ptr->wait_start) >= pos_ptr->wait_time) {
			on_goal.goal_id = pos_ptr->g_id;
			on_goal.event = ros::Time::now();
			goal_feedback.publish(on_goal);
			pos_ptr->waiting = false;
			return true;
		}
	}
	return false;
}

void checkBattery(void) {
	
	if(battery <= BATTERY_MID && battery_status == 3) {
		ROS_INFO("50 Percent Battery Remaining: %f V", battery);
		battery_status = 2;	
	}else if(battery <= BATTERY_EMPTY && battery_status == 2) {
		ROS_INFO("10 Percent Battery Remaining: %f V, land now!!", battery);
		battery_status = 1;
	}
}

void configCallback(const pc_asctec_sim::tunerConfig &config, uint32_t level) {

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

void goalCallback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{
	pos_data.g_x = msg->x;
	pos_data.g_y = msg->y;
	pos_data.g_z = msg->z;
	pos_data.g_yaw = msg->yaw;

	pos_data.g_vx = msg->vx;
	pos_data.g_vy = msg->vy;
	pos_data.g_vz = msg->vz;
	pos_data.g_vyaw = msg->vyaw;

	pos_data.g_ax = msg->ax;
	pos_data.g_ay = msg->ay;
	pos_data.g_az = msg->az;
	pos_data.g_ayaw = msg->ayaw;

	pos_data.g_arrival = false;
	pos_data.waiting = false;

	pos_data.wait_time = msg->wait_time;
	pos_data.g_range = msg->goal_limit;
	pos_data.g_id = msg->goal_id;  
}

void llCallback(const pc_asctec_sim::LLStatus::ConstPtr& msg)
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

void updatePosition(struct STATE_DATA * pos_ptr, tf::StampedTransform * transform, double dt)
{
	 //Set t-1 values
	pos_ptr->x_p = pos_ptr->x;
	pos_ptr->y_p = pos_ptr->y;
	pos_ptr->z_p = pos_ptr->z;
	pos_ptr->yaw_p = pos_ptr->yaw;
	pos_ptr->vx_p = pos_ptr->vx;
	pos_ptr->vy_p = pos_ptr->vy;
	pos_ptr->vz_p = pos_ptr->vz;
	pos_ptr->vyaw_p = pos_ptr->vyaw;
	pos_ptr->past = transform->stamp_;

	 //Get new position values
	pos_ptr->yaw = tf::getYaw(transform->getRotation()) + 2*M_PI*pos_ptr->yaw_counter;
	pos_ptr->x = transform->getOrigin().x();
	pos_ptr->y = transform->getOrigin().y();
	pos_ptr->z = transform->getOrigin().z();

	//Adjust yaw counter
	float yaw_dif = pos_ptr->yaw - pos_ptr->yaw_p;
	if(abs(yaw_dif) > M_PI) {
		if(!pos_ptr->yaw_check) {
			if(yaw_dif < 0.0) {
		 		pos_ptr->yaw_counter += 1;
	 		}else {
		 		pos_ptr->yaw_counter -= 1;
			}
			pos_ptr->yaw_check = true;
		}else {
			pos_ptr->yaw_check = false;
		}
	}

	//Calculate velocity values
		pos_ptr->vx = ((pos_ptr->x) - (pos_ptr->x_p)) / dt;
		pos_ptr->vy = ((pos_ptr->y) - (pos_ptr->y_p)) / dt;
		pos_ptr->vz = ((pos_ptr->z) - (pos_ptr->z_p)) / dt;
		pos_ptr->vyaw = ((pos_ptr->yaw) - (pos_ptr->yaw_p)) / dt;
}

void updateController(struct CTL_DATA * controller_ptr, struct STATE_DATA * pos_ptr, double dt)
{
	/* ---- P calculation ---- */
	controller_ptr->e_x = (pos_ptr->g_x) - (pos_ptr->x);
	controller_ptr->e_y = (pos_ptr->g_y) - (pos_ptr->y);
	controller_ptr->e_z = (pos_ptr->g_z) - (pos_ptr->z);
	controller_ptr->e_yaw = (pos_ptr->g_yaw) - (pos_ptr->yaw);

	/* ---- I calculation ---- */
	controller_ptr->i_x += (controller_ptr->e_x) * dt;
	controller_ptr->i_y += (controller_ptr->e_y) * dt;
	controller_ptr->i_z += (controller_ptr->e_z) * dt;
	controller_ptr->i_yaw += (controller_ptr->e_yaw) * dt;

	/* ---- Vel calculation ---- */
	controller_ptr->e_vx = (pos_ptr->g_vx) - (pos_ptr->vx);
	controller_ptr->e_vy = (pos_ptr->g_vy) - (pos_ptr->vy);
	controller_ptr->e_vz = (pos_ptr->g_vz) - (pos_ptr->vz); 
	controller_ptr->e_vyaw = (pos_ptr->g_vyaw) - (pos_ptr->vyaw);

	/* ---- Acc calculation ---- */
	controller_ptr->e_ax  = (pos_ptr->g_ax) - (pos_ptr->ax);
	controller_ptr->e_ay = (pos_ptr->g_ay) - (pos_ptr->ay);
	controller_ptr->e_az = (pos_ptr->g_az) - (pos_ptr->az); 
	controller_ptr->e_yaw = (pos_ptr->g_ayaw) - (pos_ptr->ayaw);

	/* ---- Windup Prevention ---- */
	controller_ptr->i_x = limit(controller_ptr->i_x, INTEGRAL_LIMIT, 0);
	controller_ptr->i_y = limit(controller_ptr->i_y, INTEGRAL_LIMIT, 0);
	controller_ptr->i_z = limit(controller_ptr->i_z, INTEGRAL_LIMIT, 0);
	controller_ptr->i_yaw = limit(controller_ptr->i_yaw, INTEGRAL_LIMIT, 0);
}

void setCmd(struct CTL_DATA * ctl_ptr, struct STATE_DATA * pos_ptr)
{
	double roll;
	double pitch;
	double thrust;
	double yaw;

	yaw = -(k_p_yaw * (ctl_ptr->e_yaw) + 
		k_i_yaw * (ctl_ptr->i_yaw) + 
		k_d_yaw * (ctl_ptr->e_vyaw) +
		k_a_yaw * (ctl_ptr->e_yaw));

	yaw = limit(yaw, YAW_MAX, YAW_MIN);
	
	if(!isnan(M_PI * (yaw / YAW_MAX))) {
		real_cmd.yaw = M_PI * (yaw / YAW_MAX);
	}

	pitch = -((k_p_x * (ctl_ptr->e_x) + 
		k_i_x * (ctl_ptr->i_x) + 
		k_d_x * (ctl_ptr->e_vx ) + 
		k_a_x * (ctl_ptr->e_ax)) *
		cos(pos_ptr->yaw)) + 

		-((k_p_y * (ctl_ptr->e_y) + 
		k_i_y * (ctl_ptr->i_y) + 
		k_d_y * (ctl_ptr->e_vy) + 
		k_a_y * (ctl_ptr->e_ay)) *
		sin(pos_ptr->yaw)); 

	pitch = limit(pitch, PITCH_MAX, PITCH_MIN);

	if(!isnan(M_PI * (pitch / PITCH_MAX) / 24)) {
		real_cmd.pitch = M_PI * (pitch / PITCH_MAX) / 24;
	}

	roll = ((k_p_x * (ctl_ptr->e_x) + 
		k_i_x * (ctl_ptr->i_x) + 
		k_d_x * (ctl_ptr->e_vx) +
		k_a_x * (ctl_ptr->e_ax)) *
		sin(pos_ptr->yaw)) +

		-((k_p_y * (ctl_ptr->e_y) + 
		k_i_y * (ctl_ptr->i_y) + 
		k_d_y * (ctl_ptr->e_vy) + 
		k_a_y * (ctl_ptr->e_ay)) *
		cos(pos_ptr->yaw));

	roll = limit(roll, ROLL_MAX, ROLL_MIN);

	if(!isnan(M_PI * (roll / ROLL_MAX) / 24)) {
		real_cmd.roll = M_PI * (roll / ROLL_MAX) / 24;
	}

	thrust = k_p_z * (ctl_ptr->e_z) + 
		k_i_z * (ctl_ptr->i_z) + 
		k_d_z * (ctl_ptr->e_vz) +
		k_a_z * (ctl_ptr->e_az) + G_TH;

	thrust = limit(thrust, THRUST_MAX, THRUST_MIN);

	if(!isnan(thrust / THRUST_MAX)) {
		real_cmd.thrust = thrust / THRUST_MAX;
	}
}

void setAccel(struct STATE_DATA * pos_ptr)
{
	float tempx, tempy, tempz, tempyaw;
	tempx = tempy = tempz = tempyaw = 0.0;

	for(int i = 0; i < BUFFER; i++) {
		tempx += pos_ptr->ax_buf[i];
		tempy += pos_ptr->ay_buf[i];
		tempz += pos_ptr->az_buf[i];
		tempyaw += pos_ptr->ayaw_buf[i];
	}

	pos_ptr->ax = tempx / BUFFER;
	pos_ptr->ay = tempy / BUFFER;
	pos_ptr->az = tempz / BUFFER;
	pos_ptr->ayaw = tempyaw / BUFFER;
}

void publishTrail(float x, float y, float z)
{
	geometry_msgs::Point vis_trail;
	vis_trail.x = x;
	vis_trail.y = y;
	vis_trail.z = z;

	trail.points.push_back(vis_trail);
	trail_pub.publish(trail);
	trail.points.push_back(vis_trail);
}

void initTrail(string str_frame)
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
	vis_trail.x = pos_data.x;
	vis_trail.y = pos_data.y;
	vis_trail.z = pos_data.z;
	trail.points.push_back(vis_trail);
}

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "pos_controller");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<pc_asctec_sim::tunerConfig> server;
	dynamic_reconfigure::Server<pc_asctec_sim::tunerConfig>::CallbackType f;
	f = boost::bind(&configCallback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Dynamic Reconfigure Server Started");

	string name, world_frame, frame;
	ros::param::get("~world_frame", world_frame);
	ros::param::get("~name", name);
	ros::param::get("~quad_frame", frame);
	ros::param::get("~z_offset", z_offset);

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

	pc_goal_cmd = nh.subscribe(name + "/pos_goals", 1, goalCallback);
	bat_sub = nh.subscribe(name + "/ll_status", 1, llCallback);
	ros::Rate rate(CONTROL_RATE);

	tf::StampedTransform transform;
	tf::TransformListener listener;

	CTL_DATA * controller_ptr = &ctl_data;
	STATE_DATA * pos_ptr = &pos_data;

	init(pos_ptr, controller_ptr);
	initTrail(world_frame);
	ros::Duration(1.5).sleep();
	listener.waitForTransform(world_frame, frame, ros::Time(0), ros::Duration(3.0));
	ros::spinOnce();
	if(!setVStatus()) {
		ros::spin();
	}

	listener.lookupTransform(world_frame, frame, ros::Time(0), transform);
	pos_ptr->x = transform.getOrigin().x();
	pos_ptr->y = transform.getOrigin().y();
	pos_ptr->z = transform.getOrigin().z();  
	pos_ptr->yaw = tf::getYaw(transform.getRotation());

	pos_ptr->x_p = transform.getOrigin().x();
	pos_ptr->y_p = transform.getOrigin().y();
	pos_ptr->z_p = transform.getOrigin().z(); 
	pos_ptr->yaw_p = tf::getYaw(transform.getRotation());

	pos_ptr->g_x = transform.getOrigin().x();
	pos_ptr->g_y = transform.getOrigin().y();
	
	now = transform.stamp_;
	past = now;

	double timer_past, timer = 0;
	bool check = false;
	ROS_INFO("Controller active!!");

	while (ros::ok()) {
	 //Check Battery Life
		checkBattery();
	 
	 //Grab new transform data
		listener.lookupTransform(world_frame, frame, ros::Time(0), transform);
		now = transform.stamp_;
		double dt = (now.toSec() - past.toSec());

	//Calculate accel values
		pos_ptr->ax_buf[pos_ptr->ax_n] = ((pos_ptr->vx) - (pos_ptr->vx_p)) / dt;
		pos_ptr->ay_buf[pos_ptr->ay_n] = ((pos_ptr->vy) - (pos_ptr->vy_p)) / dt;
		pos_ptr->az_buf[pos_ptr->az_n] = ((pos_ptr->vz) - (pos_ptr->vz_p)) / dt;
		pos_ptr->ayaw_buf[pos_ptr->ayaw_n] = ((pos_ptr->vyaw) - (pos_ptr->vyaw_p)) / dt;

		pos_ptr->ax_n++;
		pos_ptr->ay_n++;
		pos_ptr->az_n++;
		pos_ptr->ayaw_n++;

		pos_ptr->ax_n &= BUFFER-1;
		pos_ptr->ay_n &= BUFFER-1;
		pos_ptr->az_n &= BUFFER-1;
		pos_ptr->ayaw_n &= BUFFER-1;	

		setAccel(pos_ptr);

	//Update controller values
		updateController(controller_ptr, pos_ptr, dt);	

	//Check if target goal was accomplished -> update new goal and publish goal confirm  
		checkGoal(controller_ptr, pos_ptr);

		if(running) {
			setCmd(controller_ptr, pos_ptr);
			accel_quad_cmd.publish(real_cmd);
		}

	//Publish trail
		publishTrail(pos_ptr->x, pos_ptr->y, pos_ptr->z);

	//Fill state data and Publish
		state.event = now;

		state.x = pos_ptr->x;
		state.vx = pos_ptr->vx;
		state.ax = pos_ptr->ax;

		state.y = pos_ptr->y;
		state.vy = pos_ptr->vy;
		state.ay = pos_ptr->ay;

		state.z = pos_ptr->z;
		state.vz = pos_ptr->vz;
		state.az = pos_ptr->az;

		state.yaw = pos_ptr->yaw;
		state.vyaw = pos_ptr->vyaw;
		state.ayaw = pos_ptr->ayaw;

		state_pub.publish(state);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
