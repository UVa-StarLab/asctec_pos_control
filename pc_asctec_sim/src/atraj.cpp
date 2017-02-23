#include <atraj.h>

Trajectory_Accel::Trajectory_Accel(float d_t)
{
 /* Trajectory class constructor
  * - Holds A, B, X, T matricies
  * - Can return direct X matrix constants
  * - Can return next waypoint upon call
  */

	A.resize(6,6);
	B.resize(24,24);
	X.resize(24,24);
	T.resize(2,24);

	initAMatrix();
	initBMatrix();
	initMembers();

	dt = d_t;
}

Trajectory_Accel::~Trajectory_Accel() {}

bool Trajectory_Accel::getComplete()
{
	return isComplete;
}

struct WAYPOINT * Trajectory_Accel::updateWaypoint(struct WAYPOINT * goal)
{
 /* Update next goal point in path
  * - Sets goal based on currently running trajectory
  */

	if(point == points && !isComplete) {
		ROS_INFO("Trajectory Completed!");
		if(setBufferedPath()) {
			ROS_INFO("Buffered path set, executing...");
		}else {
			ROS_INFO("Buffer empty, waiting...");
			goal->isValid = false;
		}
	}

	if(!isComplete) {
		goal->isValid = true;
		pc_asctec_sim::pc_goal_cmd temp;
		goal->goal = temp;

		if(!isDelayed) {
			c_time += dt;
		}
		goal->goal.goal_id = c_time;

		for(int i=5; i>=0; i--) {
			goal->goal.x += X(5-i,point)*pow(c_time,i);
			goal->goal.vx += i*X(5-i,point)*pow(c_time,i-1);
			goal->goal.ax += i*(i-1)*X(5-i,point)*pow(c_time,i-2);

			goal->goal.y += X(11-i,point)*pow(c_time,i);
			goal->goal.vy += i*X(11-i,point)*pow(c_time,i-1);
			goal->goal.ay += i*(i-1)*X(11-i,point)*pow(c_time,i-2);

			goal->goal.z += X(17-i,point)*pow(c_time,i);
			goal->goal.vz += i*X(17-i,point)*pow(c_time,i-1);
			goal->goal.az += i*(i-1)*X(17-i,point)*pow(c_time,i-2);

			goal->goal.yaw += X(23-i,point)*pow(c_time,i);
			goal->goal.vyaw += i*X(23-i,point)*pow(c_time,i-1);
			goal->goal.ayaw += i*(i-1)*X(23-i,point)*pow(c_time,i-2);
		}

		if((c_time + 0.001) >= T(1,point)) {
			if(T(0,point) != 0.0 && !isDelayed) {
				isDelayed = true;
				
				t0 = ros::Time::now();
				ROS_INFO("Point %i reached, waiting for %f seconds", point, T(0,point));

			}else if(isDelayed && ((ros::Time::now().toSec() - t0.toSec()) >= T(0,point))) {
				ROS_INFO("Point %i completed", point);
				point++;
				c_time = 0.0;
				isDelayed = false;

			}else if(T(0,point) == 0.0){
				ROS_INFO("Point %i reached", point);
				point++;
				c_time = 0.0;
			}
		}
	}else {
		goal->isValid = false;
	}

	return goal;
}

struct TRPY_CONSTANTS * Trajectory_Accel::getXMatrix(struct TRPY_CONSTANTS * matrix)
{
	matrix->X.C5 = X(0,point);
	matrix->X.C4 = X(1,point);
	matrix->X.C3 = X(2,point);
	matrix->X.C2 = X(3,point);
	matrix->X.C1 = X(4,point);
	matrix->X.C0 = X(5,point);

	matrix->Y.C5 = X(6,point);
	matrix->Y.C4 = X(7,point);
	matrix->Y.C3 = X(8,point);
	matrix->Y.C2 = X(9,point);
	matrix->Y.C1 = X(10,point);
	matrix->Y.C0 = X(11,point);

	matrix->Z.C5 = X(12,point);
	matrix->Z.C4 = X(13,point);
	matrix->Z.C3 = X(14,point);
	matrix->Z.C2 = X(15,point);
	matrix->Z.C1 = X(16,point);
	matrix->Z.C0 = X(17,point);

	matrix->YAW.C5 = X(18,point);
	matrix->YAW.C4 = X(19,point);
	matrix->YAW.C3 = X(20,point);
	matrix->YAW.C2 = X(21,point);
	matrix->YAW.C1 = X(22,point);
	matrix->YAW.C0 = X(23,point);

	return matrix;
}

void Trajectory_Accel::initMembers()
{
 /* Setup class member variables
  */

	c_time = 0.0;
	point = 0;
	points = 0;

	isComplete = true;
	isDelayed = false;
}

void Trajectory_Accel::initAMatrix()
{
 /* Setup A Matrix constants
  * - Called only once in constructor
  */

	for(int i = 0; i < 6; i++) {
		for(int k = 0; k < 6; k++) {
			A(i,k) = 0.0;
		}
	}

	A(0,5) = 1.0;
	A(2,4) = 1.0;
	A(4,3) = 2.0; 
}

void Trajectory_Accel::initBMatrix()
{
 /* Setup B Matrix buffer
  * - Start B Matrix current val
  * - Called only once in constructor
  */

	B_buffer.buf_now = 0;
	for(int i=0; i<BUF; i++) {
		B_buffer.isValid[i] = false;
	}
}

void Trajectory_Accel::calcAMatrix(float time)
{
 /* Set A matrix values
  * - Set matrix positions based on spline duration
  */

	for(int i=5; i>=0; i--) {
		A(1,5-i) = pow(time,i);
		A(3,5-i) = i*pow(time,i-1);
		A(5,5-i) = i*(i-1)*pow(time,i-2);
	}
}

void Trajectory_Accel::solveXMatrix()
{
 /* Solves inverse matrix for X
  * - Uses eigen/dense library
  * - Prints out calculation time
  */

	ros::Time t_s = ros::Time::now();
	for(int i = 0; i < points; i++) {
		calcAMatrix(T(1,i));
		for(int k = 0; k < 4; k++) {
			X.block<6,1>(6*k,i) = A.colPivHouseholderQr().solve(B.block<6,1>(6*k,i));
		}
	}
	ros::Time t_e = ros::Time::now();
	float t_f = ((t_e - t_s).toNSec());
	ROS_INFO("Trajectory calculation took %f ms", t_f/1000000);

}

enum set_result Trajectory_Accel::setBMatrix(struct NEW_PATH * path)
{
 /* Sets the BMatrix or BMatrix buffer based on command type
  * - newonly writes if last trajectory is completed
  * - overwrite writes over existing trajectory
  * - buffer writes to next open buffer space
  */

	if((path->type == overwrite) || (path->type == newonly && isComplete == true) || (path->type == buffer && nextBuffer() == -1 && isComplete)) {
		points = path->cmd.points;
		for(int i = 0; i < points; i++) {
			B(1,i) = path->cmd.x[i];
			B(3,i) = path->cmd.vx[i];
			B(5,i) = path->cmd.ax[i];
		
			B(7,i) = path->cmd.y[i];
			B(9,i) = path->cmd.vy[i];
			B(11,i) = path->cmd.ay[i];

			B(13,i) = path->cmd.z[i];
			B(15,i) = path->cmd.vz[i];
			B(17,i) = path->cmd.az[i];

			B(19,i) = path->cmd.yaw[i];
			B(21,i) = path->cmd.vyaw[i];
			B(23,i) = path->cmd.ayaw[i];

			T(0,i) = path->cmd.wait_time[i];
			T(1,i) = path->cmd.duration[i];
	
		}
		B(0,0) = path->state.x;
		B(2,0) = path->state.vx;
		B(4,0) = path->state.ax;
	
		B(6,0) = path->state.y;
		B(8,0) = path->state.vy;
		B(10,0) = path->state.ay;
	
		B(12,0) = path->state.z;
		B(14,0) = path->state.vz;
		B(16,0) = path->state.az;
		
		B(18,0) = path->state.yaw;
		B(20,0) = path->state.vyaw;
		B(22,0) = path->state.ayaw;
	
		for(int i = 1; i < points; i++) {
			for(int j = 0; j < 4; j++) {
				B(6*j,i) = B(6*j+1,i-1);
				B(6*j+2,i) = B(6*j+3,i-1);
				B(6*j+4,i) = B(6*j+5,i-1);      
			}
		} 	

		solveXMatrix();

		point = 0;
		c_time = 0.0;
		isComplete = false;		


		if(path->type == overwrite && !isComplete) {
			ROS_INFO("Last trajectory overwritten!");

		}else if(path->type == newonly || path->type == overwrite) {
			ROS_INFO("New trajectory set!");
		
		}else if(path->type == buffer) {
			ROS_INFO("Buffer empty, running trajectory!");
		}
		return success;

	}else if(path->type == newonly && isComplete == false) {
		ROS_INFO("Trajectory rejected, trajectory already running!");
		return stillrunning;

	}else if(path->type == buffer && !isBufferFull()) {
		ROS_INFO("Trajectory added to buffer");
		B_buffer.cmd[nextEmptyBuffer()] = path->cmd;
		B_buffer.isValid[nextEmptyBuffer()] = true;
		return bufferadded;
		
	}else if(path->type == buffer && isBufferFull()) {
		ROS_INFO("Buffer full!");
		return bufferfull;
	}
}

bool Trajectory_Accel::setBufferedPath()
{
 /* Sets the BMatrix to the next buffered trajectory
  * - Automatically recalculates A and X matricies
  */
	if(nextBuffer() == -1) {
		isComplete = true;
		return false;

	}else {
		B(0,0) = B(1,points);
		B(2,0) = B(3,points);
		B(4,0) = B(5,points);
	
		B(6,0) = B(7,points);
		B(8,0) = B(9,points);
		B(10,0) = B(11,points);
	
		B(12,0) = B(13,points);
		B(14,0) = B(15,points);
		B(16,0) = B(17,points);
		
		B(18,0) = B(19,points);
		B(20,0) = B(21,points);
		B(22,0) = B(23,points);

		int nextCmd = nextBuffer();
		B_buffer.buf_now = nextCmd;
		B_buffer.isValid[nextCmd] = false;
		points = B_buffer.cmd[nextCmd].points;
		for(int i = 0; i < points; i++) {
			B(1,i) = B_buffer.cmd[nextCmd].x[i];	
			B(3,i) = B_buffer.cmd[nextCmd].vx[i];
			B(5,i) = B_buffer.cmd[nextCmd].ax[i];
		
			B(7,i) = B_buffer.cmd[nextCmd].y[i];
			B(9,i) = B_buffer.cmd[nextCmd].vy[i];
			B(11,i) = B_buffer.cmd[nextCmd].ay[i];

			B(13,i) = B_buffer.cmd[nextCmd].z[i];
			B(15,i) = B_buffer.cmd[nextCmd].vz[i];
			B(17,i) = B_buffer.cmd[nextCmd].az[i];

			B(19,i) = B_buffer.cmd[nextCmd].yaw[i];
			B(21,i) = B_buffer.cmd[nextCmd].vyaw[i];
			B(23,i) = B_buffer.cmd[nextCmd].ayaw[i];

			T(0,i) = B_buffer.cmd[nextCmd].wait_time[i];
			T(1,i) = B_buffer.cmd[nextCmd].duration[i];
		}
	
		for(int i = 1; i < points; i++) {
			for(int j = 0; j < 4; j++) {
				B(6*j,i) = B(6*j+1,i-1);
				B(6*j+2,i) = B(6*j+3,i-1);
				B(6*j+4,i) = B(6*j+5,i-1);      
			}
		} 
		
		solveXMatrix();

		point = 0;
		c_time = 0.0;
		isComplete = false;		
	}
	return true;
}

bool Trajectory_Accel::isBufferFull()
{
 /* Buffer checker
  * - Returns true if buffer is full
  */

	for(int i=0; i<BUF; i++) {
		if(!B_buffer.isValid[i]) {
			return false;
		}
	}

	return true;
}

int Trajectory_Accel::nextBuffer()
{
 /* Returns the position of the next buffered command
  * - If buffer is empty, then nextBuffer() returns -1
  */

	int valid = -1;
	for(int i=B_buffer.buf_now; i<BUF; i++) {
		if(B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	for(int i=0; i<B_buffer.buf_now; i++) {
		if(B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	return valid;
}

int Trajectory_Accel::nextEmptyBuffer()
{
 /* Returns the position of the next empty buffer space
  * - If buffer is full, returns -1
  */

	int valid = -1;
	for(int i=B_buffer.buf_now; i<BUF; i++) {
		if(!B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	for(int i=0; i<B_buffer.buf_now; i++) {
		if(!B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	return valid;
}
