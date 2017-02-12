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
#include <pc_asctec_sim/tunerConfig.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>

#include <string.h>
#include <math.h>

#define CONTROL_RATE 50.0
#define BUFFER 16

#define INTEGRAL_LIMIT 5.0

#define THRUST_MAX 100.0
#define THRUST_MIN 0.0
#define G_TH 30.0
#define G 9.81
#define MASS 0.7 //in kg
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

typedef struct CTL_DATA
{
	float e_x, e_vx, e_ax, i_x;
	float e_y, e_vy, e_ay, i_y;
	float e_z, e_vz, e_az, i_z;
	float e_yaw, e_vyaw, e_ayaw, i_yaw;

	float kpx, kix, kvx, kax;
	float kpy, kiy, kvy, kay;
	float kpz, kiz, kvz, kaz;
	float kpyaw, kiyaw, kvyaw, kayaw;

	float thrust, roll, pitch, yaw;

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

/* ---------------- Prototype Definitions ---------------- */

void init(struct STATE_DATA * pos_ptr, struct CTL_DATA * ctl_ptr);
void update_controller(struct CTL_DATA * ctl_ptr, struct STATE_DATA * pos_ptr);
void update_real_cmd(struct CTL_DATA * ctl_ptr, struct STATE_DATA * pos_ptr);
void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg);
bool goal_arrived(struct CTL_DATA * ctl_ptr, struct STATE_DATA * pos_ptr);
float limit(float input, float ceiling, float floor);
