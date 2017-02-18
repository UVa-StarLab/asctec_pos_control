#ifndef TRAJECTORYACCEL_H
#define TRAJECTORYACCEL_H

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>

#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#define BUF 3

using namespace std;
using Eigen::MatrixXd;


/* -------------------- Enum Definitions ---------------- */

enum set_result
{
success,
stillrunning,
bufferadded,
bufferfull
};

enum path_type
{
newonly,
overwrite,
buffer
};

/* -------------------- Struct Definitions ---------------- */
typedef struct CONSTANTS
{
	float C5, C4, C3, C2, C1, C0;

}constants;

typedef struct TRPY_CONSTANTS
{
	struct CONSTANTS X,Y,Z,YAW;

}trpy_constants;

typedef struct NEW_PATH
{
	path_type type;
	pc_asctec_sim::pc_state state;
	pc_asctec_sim::pc_traj_cmd cmd;

}new_path;

typedef struct WAYPOINT
{
	bool isValid;
	pc_asctec_sim::pc_goal_cmd goal;

}waypoint;

typedef struct CMD_BUF
{
	pc_asctec_sim::pc_traj_cmd cmd[BUF];
	bool isValid[BUF];
	int buf_now;

}cmd_buf;

/* -------------------- Class Definition ---------------- */
class Trajectory_Accel
{
	public:
		Trajectory_Accel(float d_t);
		~Trajectory_Accel();

		bool getComplete();			
		bool getDelayed();
		double getDelayTime();
		void setDelayed(bool val);
		void setStarted(bool val);

		struct WAYPOINT * updateWaypoint(struct WAYPOINT * goal);
		struct TRPY_CONSTANTS * getXMatrix(struct TRPY_CONSTANTS * matrix);
		enum set_result setBMatrix(struct NEW_PATH * path);

	private:
		void initMembers();
		void initAMatrix();
		void initBMatrix();

		void calcAMatrix(float time);
		void solveXMatrix();

		bool setBufferedPath();
		bool isBufferFull();	
		int nextBuffer();
		int nextEmptyBuffer();

		MatrixXd A, B, X, T;	//6x6, 24x24, 24x24, 2x24
		struct CMD_BUF B_buffer;

		float dt;
		double c_time;
		double delay_time;
		int point;
		int points;
		bool isComplete;
		bool isDelayed;
		bool isStarted;
};
#endif