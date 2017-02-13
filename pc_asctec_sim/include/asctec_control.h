#ifndef ASCTECCONTROL_H
#define ASCTECCONTROL_H

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <pc_asctec_sim/SICmd.h>
#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_state.h>
#include <pc_asctec_sim/LLStatus.h>
#include <pc_asctec_sim/ascTunerConfig.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>

#include <string.h>
#include <math.h>

using namespace std;

#define CONTROL_RATE 50.0
#define BUFFER 16

#define INTEGRAL_LIMIT 5.0

#define MASS 0.7 //in kg
#define G_TH 25.0
#define G 9.81

#define BOUNDED_ANGLE 24.0 // divided by this number

#define THRUST_MAX 100.0
#define THRUST_MIN 0.0

#define ROLL_MAX 100.0
#define ROLL_MIN -ROLL_MAX

#define PITCH_MAX 100.0
#define PITCH_MIN -PITCH_MAX

#define YAW_MAX 100.0
#define YAW_MIN -YAW_MAX

#define BATTERY_FULL 12.35
#define BATTERY_MID 10.5
#define BATTERY_EMPTY 9.5

using namespace std;

/* --------------- Data Structure Definitions ------------ */

typedef struct K_DATA 
{
	float kpx, kix, kvx, kax;
	float kpy, kiy, kvy, kay;
	float kpz, kiz, kvz, kaz;
	float kpyaw, kiyaw, kvyaw, kayaw;
}k_data;

typedef struct PUB_DATA 
{
	pc_asctec_sim::pc_state state;
	pc_asctec_sim::pc_feedback g_feedback;
	pc_asctec_sim::SICmd TRPYcmd;

	struct K_DATA k_val;
	bool running;
	float battery;

}pub_data;

typedef struct GOAL_DATA 
{
	bool isNew;
	pc_asctec_sim::pc_goal_cmd goal;

} goal_data;

typedef struct CTL_DATA
{
	float e_x, e_vx, e_ax, i_x;
	float e_y, e_vy, e_ay, i_y;
	float e_z, e_vz, e_az, i_z;
	float e_yaw, e_vyaw, e_ayaw, i_yaw;

	struct K_DATA k_val;

} control_data;

typedef struct STATE_DATA
{
	double dt;

	float x, y, z, yaw, relative_yaw;
	float x_p, y_p, z_p, yaw_p;
	float vx, vy, vz, vyaw;
	float vx_p, vy_p, vz_p, vyaw_p;

	float ax, ay, az, ayaw;
	int ax_n, ay_n, az_n, ayaw_n;
	float ax_buf[BUFFER], ay_buf[BUFFER], az_buf[BUFFER], ayaw_buf[BUFFER];

	float g_x, g_y, g_z, g_yaw;
	float g_vx, g_vy, g_vz, g_vyaw;
	float g_ax, g_ay, g_az, g_ayaw;
	float g_range;
	float wait_time, wait_start;
	
	float battery;
	int battery_status;
	int yaw_counter;
	bool yaw_check, waiting, g_arrival;
	string g_id;
	ros::Time past;

} state_data;


/* -------------------- Class Definition ---------------- */
class AscTec_Controller
{
	public:
		AscTec_Controller();
		~AscTec_Controller();
		struct PUB_DATA * runAsctec(struct PUB_DATA * pub, struct GOAL_DATA * goal_in, tf::StampedTransform * transform);
		void setParams(struct K_DATA * k_ptr);
	private:

		void updatePosition(struct STATE_DATA * st_ptr, tf::StampedTransform * transform);
		void freeGoal(struct STATE_DATA * st_ptr);
		void updateGoal(struct GOAL_DATA * g_ptr, struct STATE_DATA * st_ptr);
		void updateController(struct CTL_DATA * ctl_ptr, struct STATE_DATA * st_ptr);
		pc_asctec_sim::SICmd * setCmd(struct CTL_DATA * ctl_ptr, struct STATE_DATA * st_ptr, pc_asctec_sim::SICmd * TRPY);
		pc_asctec_sim::pc_state * fillState(struct STATE_DATA * st_ptr, pc_asctec_sim::pc_state * out_ptr);
		void initAsctec(struct STATE_DATA * pos_ptr, struct CTL_DATA * ctl_ptr);
		
		void checkBattery(struct STATE_DATA * state_ptr);
		pc_asctec_sim::pc_feedback * checkGoal(struct CTL_DATA * ctl_ptr, struct STATE_DATA * st_ptr, pc_asctec_sim::pc_feedback * on_goal);
		float limitOutput(float input, float ceiling, float floor);

		float battery;
		CTL_DATA controller, *c_ptr;
		STATE_DATA state, *s_ptr;
};
#endif
