#include <asctec_control.h>

AscTec_Controller::AscTec_Controller()
{	
	c_ptr = &controller;
	s_ptr = &state;
	battery = BATTERY_FULL;
	initAsctec(s_ptr, c_ptr);
}

AscTec_Controller::~AscTec_Controller() {}

struct PUB_DATA * AscTec_Controller::runAsctec(struct PUB_DATA * pub, struct GOAL_DATA * goal_in, tf::StampedTransform * transform)
{
	/* ---- Update State ---- */
	AscTec_Controller::updatePosition(s_ptr, transform);
	pub->state = *fillState(s_ptr, &pub->state);

	/* ---- Update Params and Status ---- */
	s_ptr->battery = pub->battery;
	AscTec_Controller::checkBattery(s_ptr);
	pub->g_feedback = *checkGoal(c_ptr, s_ptr, &pub->g_feedback);
	c_ptr->k_val = pub->k_val;

	/* ---- Update Current Goal ---- */
	if(pub->running) {
		updateGoal(goal_in, s_ptr);
	}else {
		freeGoal(s_ptr);
	}

	/* ---- Update Controller ---- */
	updateController(c_ptr, s_ptr);

	/* ---- Set TRPY Command ---- */
	if(pub->running) {
		pub->TRPYcmd = *setCmd(c_ptr, s_ptr, &pub->TRPYcmd);
	}else {
		pub->TRPYcmd.cmd[0] = true;
		pub->TRPYcmd.cmd[1] = true;
		pub->TRPYcmd.cmd[2] = true;
		pub->TRPYcmd.cmd[3] = true;
		pub->TRPYcmd.thrust = 0;
	}

	return pub;
}

void AscTec_Controller::setParams(struct K_DATA * k_ptr)
{
	c_ptr->k_val = *k_ptr;
}

void AscTec_Controller::updatePosition(struct STATE_DATA * st_ptr, tf::StampedTransform * transform)
{
	st_ptr->dt = transform->stamp_.toSec() - st_ptr->past.toSec();

	//Set t-1 values
	st_ptr->x_p = st_ptr->x;
	st_ptr->y_p = st_ptr->y;
	st_ptr->z_p = st_ptr->z;
	st_ptr->yaw_p = st_ptr->yaw;
	st_ptr->vx_p = st_ptr->vx;
	st_ptr->vy_p = st_ptr->vy;
	st_ptr->vz_p = st_ptr->vz;
	st_ptr->vyaw_p = st_ptr->vyaw;
	st_ptr->past = transform->stamp_;

	//Get new position values
	st_ptr->yaw = tf::getYaw(transform->getRotation()) + 2*M_PI*st_ptr->yaw_counter;
	st_ptr->x = transform->getOrigin().x();
	st_ptr->y = transform->getOrigin().y();
	st_ptr->z = transform->getOrigin().z();

	//Adjust yaw counter
	float yaw_dif = st_ptr->yaw - st_ptr->yaw_p;
	if(abs(yaw_dif) > M_PI) {
		if(!st_ptr->yaw_check) {
			if(yaw_dif < 0.0) {
		 		st_ptr->yaw_counter += 1;
	 		}else {
		 		st_ptr->yaw_counter -= 1;
			}
			st_ptr->yaw_check = true;
		}else {
			st_ptr->yaw_check = false;
		}
	}

	//Calculate velocity values
	st_ptr->vx = ((st_ptr->x) - (st_ptr->x_p)) / st_ptr->dt;
	st_ptr->vy = ((st_ptr->y) - (st_ptr->y_p)) / st_ptr->dt;
	st_ptr->vz = ((st_ptr->z) - (st_ptr->z_p)) / st_ptr->dt;
	st_ptr->vyaw = ((st_ptr->yaw) - (st_ptr->yaw_p)) / st_ptr->dt;

	//Calculate accel values
	st_ptr->ax_buf[st_ptr->ax_n] = ((st_ptr->vx) - (st_ptr->vx_p)) / st_ptr->dt;
	st_ptr->ay_buf[st_ptr->ay_n] = ((st_ptr->vy) - (st_ptr->vy_p)) / st_ptr->dt;
	st_ptr->az_buf[st_ptr->az_n] = ((st_ptr->vz) - (st_ptr->vz_p)) / st_ptr->dt;
	st_ptr->ayaw_buf[st_ptr->ayaw_n] = ((st_ptr->vyaw) - (st_ptr->vyaw_p)) / st_ptr->dt;

	st_ptr->ax_n++;
	st_ptr->ay_n++;
	st_ptr->az_n++;
	st_ptr->ayaw_n++;

	st_ptr->ax_n &= BUFFER-1;
	st_ptr->ay_n &= BUFFER-1;
	st_ptr->az_n &= BUFFER-1;
	st_ptr->ayaw_n &= BUFFER-1;

	float tempx, tempy, tempz, tempyaw;
	tempx = tempy = tempz = tempyaw = 0.0;

	for(int i = 0; i < BUFFER; i++) {
		tempx += st_ptr->ax_buf[i];
		tempy += st_ptr->ay_buf[i];
		tempz += st_ptr->az_buf[i];
		tempyaw += st_ptr->ayaw_buf[i];
	}

	st_ptr->ax = tempx / BUFFER;
	st_ptr->ay = tempy / BUFFER;
	st_ptr->az = tempz / BUFFER;
	st_ptr->ayaw = tempyaw / BUFFER;
}

void AscTec_Controller::freeGoal(struct STATE_DATA * st_ptr)
{
	st_ptr->g_x = st_ptr->x;
	st_ptr->g_y = st_ptr->y;
	st_ptr->g_z = st_ptr->z;
	st_ptr->g_yaw = st_ptr->yaw;

	st_ptr->g_vx = 0;
	st_ptr->g_vy = 0;
	st_ptr->g_vz = 0;
	st_ptr->g_vyaw = 0;

	st_ptr->g_ax = 0;
	st_ptr->g_ay = 0;
	st_ptr->g_az = 0;
	st_ptr->g_ayaw = 0;

	st_ptr->g_range = 0;
	st_ptr->g_id = "init";
	st_ptr->wait_time = 0;
}

void AscTec_Controller::updateGoal(struct GOAL_DATA * g_ptr, struct STATE_DATA * st_ptr)
{
	if(g_ptr->isNew) {
		st_ptr->g_x = g_ptr->goal.x;
		st_ptr->g_y = g_ptr->goal.y;
		st_ptr->g_z = g_ptr->goal.z;
		st_ptr->g_yaw = g_ptr->goal.yaw;

		st_ptr->g_vx = g_ptr->goal.vx;
		st_ptr->g_vy = g_ptr->goal.vy;
		st_ptr->g_vz = g_ptr->goal.vz;
		st_ptr->g_vyaw = g_ptr->goal.vyaw;

		st_ptr->g_ax = g_ptr->goal.ax;
		st_ptr->g_ay = g_ptr->goal.ay;
		st_ptr->g_az = g_ptr->goal.az;
		st_ptr->g_ayaw = g_ptr->goal.ayaw;

		st_ptr->g_range = g_ptr->goal.goal_limit;
		st_ptr->g_id = g_ptr->goal.goal_id;
		st_ptr->wait_time = g_ptr->goal.wait_time;
	}
}

void AscTec_Controller::updateController(struct CTL_DATA * ctl_ptr, struct STATE_DATA * st_ptr)
{
	/* ---- P calculation ---- */
	ctl_ptr->e_x = (st_ptr->g_x) - (st_ptr->x);
	ctl_ptr->e_y = (st_ptr->g_y) - (st_ptr->y);
	ctl_ptr->e_z = (st_ptr->g_z) - (st_ptr->z);
	ctl_ptr->e_yaw = (st_ptr->g_yaw) - (st_ptr->yaw);

	/* ---- I calculation ---- */
	ctl_ptr->i_x += (ctl_ptr->e_x) * st_ptr->dt;
	ctl_ptr->i_y += (ctl_ptr->e_y) * st_ptr->dt;
	ctl_ptr->i_z += (ctl_ptr->e_z) * st_ptr->dt;
	ctl_ptr->i_yaw += (ctl_ptr->e_yaw) * st_ptr->dt;

	/* ---- Vel calculation ---- */
	ctl_ptr->e_vx = (st_ptr->g_vx) - (st_ptr->vx);
	ctl_ptr->e_vy = (st_ptr->g_vy) - (st_ptr->vy);
	ctl_ptr->e_vz = (st_ptr->g_vz) - (st_ptr->vz); 
	ctl_ptr->e_vyaw = (st_ptr->g_vyaw) - (st_ptr->vyaw);

	/* ---- Acc calculation ---- */
	ctl_ptr->e_ax  = (st_ptr->g_ax) - (st_ptr->ax);
	ctl_ptr->e_ay = (st_ptr->g_ay) - (st_ptr->ay);
	ctl_ptr->e_az = (st_ptr->g_az) - (st_ptr->az); 
	ctl_ptr->e_ayaw = (st_ptr->g_ayaw) - (st_ptr->ayaw);

	/* ---- Windup Prevention ---- */
	ctl_ptr->i_x = limitOutput(ctl_ptr->i_x, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
	ctl_ptr->i_y = limitOutput(ctl_ptr->i_y, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
	ctl_ptr->i_z = limitOutput(ctl_ptr->i_z, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
	ctl_ptr->i_yaw = limitOutput(ctl_ptr->i_yaw, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
}

pc_asctec_sim::SICmd * AscTec_Controller::setCmd(struct CTL_DATA * ctl_ptr, struct STATE_DATA * st_ptr, pc_asctec_sim::SICmd * TRPY)
{
	double roll;
	double pitch;
	double thrust;
	double yaw;

	TRPY->cmd[0] = true;
	TRPY->cmd[1] = true;
	TRPY->cmd[2] = true;
	TRPY->cmd[3] = true;
	
	yaw = -(ctl_ptr->k_val.kpyaw * (ctl_ptr->e_yaw) + 
		ctl_ptr->k_val.kiyaw * (ctl_ptr->i_yaw) + 
		ctl_ptr->k_val.kvyaw * (ctl_ptr->e_vyaw) +
		ctl_ptr->k_val.kayaw * (ctl_ptr->e_ayaw));

	yaw = limitOutput(yaw, YAW_MAX, YAW_MIN);
	
	if(!isnan(M_PI * (yaw / YAW_MAX))) {
		TRPY->yaw = M_PI * (yaw / YAW_MAX);
	}

	pitch = -((ctl_ptr->k_val.kpx * (ctl_ptr->e_x) + 
		ctl_ptr->k_val.kix * (ctl_ptr->i_x) + 
		ctl_ptr->k_val.kvx * (ctl_ptr->e_vx ) + 
		ctl_ptr->k_val.kax * (ctl_ptr->e_ax)) *
		cos(st_ptr->yaw)) + 

		-((ctl_ptr->k_val.kpy * (ctl_ptr->e_y) + 
		ctl_ptr->k_val.kiy * (ctl_ptr->i_y) + 
		ctl_ptr->k_val.kvy * (ctl_ptr->e_vy) + 
		ctl_ptr->k_val.kay * (ctl_ptr->e_ay)) *
		sin(st_ptr->yaw)); 

	pitch = limitOutput(pitch, PITCH_MAX, PITCH_MIN);

	if(!isnan(M_PI * (pitch / PITCH_MAX) / BOUNDED_ANGLE)) {
		TRPY->pitch = M_PI * (pitch / PITCH_MAX) / BOUNDED_ANGLE;
	}

	roll = ((ctl_ptr->k_val.kpx * (ctl_ptr->e_x) + 
		ctl_ptr->k_val.kix * (ctl_ptr->i_x) + 
		ctl_ptr->k_val.kvx * (ctl_ptr->e_vx) +
		ctl_ptr->k_val.kax * (ctl_ptr->e_ax)) *
		sin(st_ptr->yaw)) +

		-((ctl_ptr->k_val.kpy * (ctl_ptr->e_y) + 
		ctl_ptr->k_val.kiy * (ctl_ptr->i_y) + 
		ctl_ptr->k_val.kvy * (ctl_ptr->e_vy) + 
		ctl_ptr->k_val.kay * (ctl_ptr->e_ay)) *
		cos(st_ptr->yaw));

	roll = limitOutput(roll, ROLL_MAX, ROLL_MIN);

	if(!isnan(M_PI * (roll / ROLL_MAX) / BOUNDED_ANGLE)) {
		TRPY->roll = M_PI * (roll / ROLL_MAX) / BOUNDED_ANGLE;
	}

	thrust = ctl_ptr->k_val.kpz * (ctl_ptr->e_z) + 
		ctl_ptr->k_val.kiz * (ctl_ptr->i_z) + 
		ctl_ptr->k_val.kvz * (ctl_ptr->e_vz) +
		ctl_ptr->k_val.kaz * (ctl_ptr->e_az) + G_TH;

	thrust = limitOutput(thrust, THRUST_MAX, THRUST_MIN);

	if(!isnan(thrust / THRUST_MAX)) {
		TRPY->thrust = thrust / THRUST_MAX;
	}

	return TRPY;
}

pc_asctec_sim::pc_state * AscTec_Controller::fillState(struct STATE_DATA * st_ptr, pc_asctec_sim::pc_state * out_ptr)
{
	out_ptr->event = st_ptr->past;

	out_ptr->x = st_ptr->x;
	out_ptr->vx = st_ptr->vx;
	out_ptr->ax = st_ptr->ax;
	
	out_ptr->y = st_ptr->y;
	out_ptr->vy = st_ptr->vy;
	out_ptr->ay = st_ptr->ay;

	out_ptr->z = st_ptr->z;
	out_ptr->vz = st_ptr->vz;
	out_ptr->az = st_ptr->az;

	out_ptr->yaw = st_ptr->yaw;
	out_ptr->vyaw = st_ptr->vyaw;
	out_ptr->ayaw = st_ptr->ayaw;
	
	return out_ptr;
}

void AscTec_Controller::initAsctec(struct STATE_DATA * st_ptr, struct CTL_DATA * ctl_ptr)
{
	/* ----- Controller Init ----- */
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

	/* ----- Position Init ----- */
	st_ptr->x = 0.0;
	st_ptr->y = 0.0;
	st_ptr->z = 0.0;
	st_ptr->yaw = 0.0;

	st_ptr->x_p = 0.0;
	st_ptr->y_p = 0.0;
	st_ptr->z_p = 0.0;
	st_ptr->yaw_p = 0.0;

	st_ptr->vx = 0.0;
	st_ptr->vy = 0.0;
	st_ptr->vz = 0.0;
	st_ptr->vyaw = 0.0;

	st_ptr->g_x = 0.0;
	st_ptr->g_y = 0.0;
	st_ptr->g_z = 0.0;
	st_ptr->g_yaw = 0.0;

	st_ptr->g_vx = 0.0;
	st_ptr->g_vy = 0.0;
	st_ptr->g_vz = 0.0;
	st_ptr->g_vyaw = 0.0;

	st_ptr->ax_n = 0;
	st_ptr->ay_n = 0;
	st_ptr->az_n = 0;
	st_ptr->ayaw_n = 0;

	st_ptr->yaw_counter = 0;

	for(int i = 0; i < BUFFER; i++) {
		st_ptr->ax_buf[i] = 0;
		st_ptr->ay_buf[i] = 0;
		st_ptr->az_buf[i] = 0;
		st_ptr->ayaw_buf[i] = 0;
	}

	st_ptr->g_range = 0.05;
	st_ptr->wait_time = 0.0;
	st_ptr->waiting = false;
	st_ptr->g_arrival = false;
	st_ptr->g_id = "Init";
}

void AscTec_Controller::checkBattery(struct STATE_DATA * state_ptr)
{
	if(state_ptr->battery <= BATTERY_MID && state_ptr->battery_status == 3) {
		ROS_INFO("50 Percent Battery Remaining: %f V", state_ptr->battery);
		state_ptr->battery_status = 2;	

	}else if(state_ptr->battery <= BATTERY_EMPTY && state_ptr->battery_status == 2) {
		ROS_INFO("10 Percent Battery Remaining: %f V, land now!!", state_ptr->battery);
		state_ptr->battery_status = 1;
	}
}

pc_asctec_sim::pc_feedback * AscTec_Controller::checkGoal(struct CTL_DATA * ctl_ptr, struct STATE_DATA * st_ptr, pc_asctec_sim::pc_feedback * on_goal)
{
	float distance = sqrt((ctl_ptr->e_x) * (ctl_ptr->e_x) + 
			(ctl_ptr->e_y) * (ctl_ptr->e_y) + 
			(ctl_ptr->e_z) * (ctl_ptr->e_z)); 

	if((distance <= (st_ptr->g_range)) && not (st_ptr->g_arrival)) {
		st_ptr->wait_start = ros::Time::now().toSec(); 
		st_ptr->waiting = true;  
		st_ptr->g_arrival = true;
	}
	if(st_ptr->waiting) {
		if((ros::Time::now().toSec() - st_ptr->wait_start) >= st_ptr->wait_time) {
			st_ptr->waiting = false;
			on_goal->goal_id = st_ptr->g_id;
			on_goal->event = ros::Time::now();
			on_goal->arrived = true;
		}
	}
	return on_goal;
}

float AscTec_Controller::limitOutput(float input, float ceiling, float floor) 
{
	if(input > ceiling) {
		return ceiling;
	}else if(input < floor) {
		return floor;
	}else {
		return input;
	}
}
