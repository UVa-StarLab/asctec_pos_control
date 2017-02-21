#include "asctec_quad.h"

AscTec_Quad::AscTec_Quad(string qframe, string wframe, float dt, struct K_DATA * kvals):
controller(qframe, wframe, kvals), accTraj(dt) 
{
	P_ptr = new PUB_DATA;
	W_ptr = new WAYPOINT;
	Q_ptr = new QUAD_OUT;
}

QUAD_OUT * AscTec_Quad::runQuad(QUAD_CMD * cmd, tf::StampedTransform * transform)
{
	P_ptr->k_val = cmd->kvals;
	P_ptr->running = cmd->start;
	P_ptr->battery = cmd->battery;

	if(cmd->stopTimer) {
		accTraj.setDelayed(false);
		accTraj.setStarted(true);
	}

	if(cmd->newPath) {
		NEW_PATH path;
		path.cmd = cmd->f_path;
		path.state = P_ptr->state;
		path.type = buffer;
		accTraj.setBMatrix(&path);
		
	}

	if(!accTraj.getComplete()) {
		if(!accTraj.getDelayed()) {
			W_ptr = accTraj.updateWaypoint(W_ptr);

			if(W_ptr->isValid) {
				goal_.goal = W_ptr->goal;
				goal_.isNew = true;
				Q_ptr->goal.x = W_ptr->goal.x;
				Q_ptr->goal.y = W_ptr->goal.y;
				Q_ptr->goal.z = W_ptr->goal.z;
			}

		}else if(accTraj.getDelayed() && !cmd->stopTimer) {
			Q_ptr->startTimer = true;
			Q_ptr->time = accTraj.getDelayTime();
		}

	}else {
		goal_.isNew = false;
	}
	P_ptr = controller.runAsctec(P_ptr,&goal_,transform);
}
