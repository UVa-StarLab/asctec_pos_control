#include <avoid.h>
#include "atraj.cpp"

Avoid::Avoid(std::string world_)
{
	path_gen = new Trajectory_Accel(1.0);
	world = world_;
	
}

Avoid::~Avoid()
{

}

void Avoid::addNewINode(double t, int spline)
{
	struct Isection * next = new Isection;
	next->t = t;
	next->spline = spline;

	if(Isec_ptr) {
		getLastINode()->next = next;
	}else {
		Isec_ptr = next;
	}
}

void Avoid::addNewONode(std::string frame, 
			float x, 
			float y, 
			float r)
{
	struct obs * next = new obs;
	obs->frame = frame;
	obs->x = x;
	obs->y = y;
	obs->r = r;
	if(obs_ptr) {
		getLastONode()->next = next;
	}else {
		obs_ptr = next;
	}
}

void deleteIList(struct Isec * node)
{
	if(node->next) {
		deleteIList(node->next)
		delete *node;
	}else {
		Isec_ptr = NULL;
	}
}

void deleteOList(struct obs * node)
{
	if(node->next) {
		deleteOList(node->next)
		delete *node;
	}else {
		obs_ptr = NULL;
	}
}

struct Isection * getLastINode(struct Isection * node)
{
	if(node->next) {
		return getLastINode(node->next);
	}else {
		return node;
	}
}

struct obs * getLastONode(struct obs * node)
{
	if(node->next) {
		return getLastONode(node->next);
	}else {
		return node;
	}
}

void Avoid::setC(pc_asctec_sim::pc_traj_cmd * path, pc_asctec_sim::pc_state * q_st)
{
	NEW_PATH temp;
	temp.state = *q_st;
	temp.cmd = *path;
	points = temp.cmd.points;
	C = *path_gen->getPathConstants(&temp,&C);
}

void Avoid::getSplineRoots(	int spline, struct obs * ob_ptr, 
				struct Isec * I_ptr, pc_asctec_sim::pc_traj_cmd * path,
				pc_asctec_sim::pc_state * q_st)
{
	setC(path, q_st);
	Eigen::MatrixXd companion = Eigen::MatrixXd::Constant(10,10,0);
	for(int i=0; i<9; i++) {
		companion(i+1,i) = 1;
	}
	double A10 = -(pow(C.c5x[spline],2) + pow(C.c5y[spline],2));

	companion(0,9) = (pow((C.c0x[spline] - ob_ptr->x),2) + pow((C.c0y[spline] - ob_ptr->y),2) - pow(ob_ptr->r,2))/A10;
	companion(1,9) = (2*C.c0x[spline]*C.c1x[spline] - 2*C.c1x[spline]*ob_ptr->x + 2*C.c0y[spline]*C.c1y[spline] - 2*C.c1y[spline]*ob_ptr->y)/A10;
	companion(2,9) = (2*C.c0x[spline]*C.c2x[spline] + pow(C.c1x[spline],2) - 2*C.c2x[spline]*ob_ptr->x + 2*C.c0y[spline]*C.c2y[spline] + pow(C.c1y[spline],2) - 2*C.c2y[spline]*ob_ptr->y)/A10;
	companion(3,9) = (2*C.c0x[spline]*C.c3x[spline] + 2*C.c1x[spline]*C.c2x[spline] - 2*C.c3x[spline]*ob_ptr->x + 2*C.c0y[spline]*C.c3y[spline] + 2*C.c1y[spline]*C.c2y[spline] - 2*C.c3y[spline]*ob_ptr->y)/A10;
	companion(4,9) = (2*C.c0x[spline]*C.c4x[spline] + 2*C.c1x[spline]*C.c3x[spline] + pow(C.c2x[spline],2) - 2*C.c4x[spline]*ob_ptr->x + 2*C.c0y[spline]*C.c4y[spline] + 2*C.c1y[spline]*C.c3y[spline] + pow(C.c2y[spline],2) - 2*C.c4y[spline]*ob_ptr->y)/A10; 
	companion(5,9) = (2*C.c0x[spline]*C.c5x[spline] + 2*C.c1x[spline]*C.c4x[spline] + 2*C.c2x[spline]*C.c3x[spline] - 2*C.c5x[spline]*ob_ptr->x + 2*C.c0y[spline]*C.c5y[spline] + 2*C.c1y[spline]*C.c4y[spline] + 2*C.c2y[spline]*C.c3y[spline] - 2*C.c5y[spline]*ob_ptr->y)/A10;
	companion(6,9) = (2*C.c1x[spline]*C.c5x[spline] + 2*C.c2x[spline]*C.c4x[spline] + pow(C.c3x[spline],2) + 2*C.c1y[spline]*C.c5y[spline] + 2*C.c2y[spline]*C.c4y[spline] + pow(C.c3y[spline],2))/A10;
	companion(7,9) = (2*C.c2x[spline]*C.c5x[spline] + 2*C.c3x[spline]*C.c4x[spline] + 2*C.c2y[spline]*C.c5y[spline] + 2*C.c3y[spline]*C.c4y[spline])/A10;
	companion(8,9) = (2*C.c3x[spline]*C.c5x[spline] + pow(C.c4x[spline],2) + 2*C.c3y[spline]*C.c5y[spline] + pow(C.c4y[spline],2))/A10;
	companion(9,9) = (2*C.c4x[spline]*C.c5x[spline] + 2*C.c4y[spline]*C.c5y[spline])/A10;

	Eigen::EigenSolver<Eigen::MatrixXd> result(companion);
	for(int k=0; k<10; k++) {
		complex<double> lambda = result.eigenvalues()[k];
		if(lambda.imag() == 0) {
			addNewINode(lambda.real(), spline);
		}
	}
}

void Avoid::getRoots(	struct obs * ob_ptr, 
			struct Isec * I_ptr, 
			pc_asctec_sim::pc_traj_cmd * path,
			pc_asctec_sim::pc_state * q_st)
{
	NEW_PATH temp;
	temp.state = *q_st;
	temp.cmd = *path;
	points = temp.cmd.points;
	C = *path_gen->getPathConstants(&temp,&C);

	Eigen::MatrixXd companion = Eigen::MatrixXd::Constant(10,10,0);
	for(int i=0; i<9; i++) {
		companion(i+1,i) = 1;
	}

	int sp = 0;
	for(int i=0; i<points; i++) {
		double A10 = -(pow(C.c5x[i],2) + pow(C.c5y[i],2));

		companion(0,9) = (pow((C.c0x[i] - ob_ptr->x),2) + pow((C.c0y[i] - ob_ptr->y),2) - pow(ob_ptr->r,2))/A10;
		companion(1,9) = (2*C.c0x[i]*C.c1x[i] - 2*C.c1x[i]*ob_ptr->x + 2*C.c0y[i]*C.c1y[i] - 2*C.c1y[i]*ob_ptr->y)/A10;
		companion(2,9) = (2*C.c0x[i]*C.c2x[i] + pow(C.c1x[i],2) - 2*C.c2x[i]*ob_ptr->x + 2*C.c0y[i]*C.c2y[i] + pow(C.c1y[i],2) - 2*C.c2y[i]*ob_ptr->y)/A10;
		companion(3,9) = (2*C.c0x[i]*C.c3x[i] + 2*C.c1x[i]*C.c2x[i] - 2*C.c3x[i]*ob_ptr->x + 2*C.c0y[i]*C.c3y[i] + 2*C.c1y[i]*C.c2y[i] - 2*C.c3y[i]*ob_ptr->y)/A10;
		companion(4,9) = (2*C.c0x[i]*C.c4x[i] + 2*C.c1x[i]*C.c3x[i] + pow(C.c2x[i],2) - 2*C.c4x[i]*ob_ptr->x + 2*C.c0y[i]*C.c4y[i] + 2*C.c1y[i]*C.c3y[i] + pow(C.c2y[i],2) - 2*C.c4y[i]*ob_ptr->y)/A10; 
		companion(5,9) = (2*C.c0x[i]*C.c5x[i] + 2*C.c1x[i]*C.c4x[i] + 2*C.c2x[i]*C.c3x[i] - 2*C.c5x[i]*ob_ptr->x + 2*C.c0y[i]*C.c5y[i] + 2*C.c1y[i]*C.c4y[i] + 2*C.c2y[i]*C.c3y[i] - 2*C.c5y[i]*ob_ptr->y)/A10;
		companion(6,9) = (2*C.c1x[i]*C.c5x[i] + 2*C.c2x[i]*C.c4x[i] + pow(C.c3x[i],2) + 2*C.c1y[i]*C.c5y[i] + 2*C.c2y[i]*C.c4y[i] + pow(C.c3y[i],2))/A10;
		companion(7,9) = (2*C.c2x[i]*C.c5x[i] + 2*C.c3x[i]*C.c4x[i] + 2*C.c2y[i]*C.c5y[i] + 2*C.c3y[i]*C.c4y[i])/A10;
		companion(8,9) = (2*C.c3x[i]*C.c5x[i] + pow(C.c4x[i],2) + 2*C.c3y[i]*C.c5y[i] + pow(C.c4y[i],2))/A10;
		companion(9,9) = (2*C.c4x[i]*C.c5x[i] + 2*C.c4y[i]*C.c5y[i])/A10;

		Eigen::EigenSolver<Eigen::MatrixXd> result(companion);
		
		int j = 0;
		for(int k=0; k<10; k++) {
			complex<double> lambda = result.eigenvalues()[k];
			if(lambda.imag() == 0) {
				ob_ptr->I[j][sp] = lambda.real();
				j++;
				ob_ptr->spline[sp] = i;
				ob_ptr->check = true;
			}
		}
		sp++;
	}
}

double Avoid::getObsAngle(struct Isec * I_ptr, int spline)
{
	float Ix1 = C.c0x[spline]*pow((I_ptr->t),5)
	float Ix2
	float Iy1
	float Iy2

	double a = sqrt(pow(Ix1 - Ix2,2) + pow(Iy1 - Iy2,2));
	double angle = acos((2*(pow(ob_ptr->r,2)) - pow(a,2))/(2*(pow(ob_ptr->r,2)));
	return angle;
}

pc_asctec_sim::pc_traj_cmd * Avoid::injectObsPoints(pc_asctec_sim::pc_traj_cmd * path)
{
	getRoots(ob_ptr, path);
	double maxAng = getObsAngle(ob_ptr);
}

pc_asctec_sim::pc_traj_cmd * AVoid::adaptTrajectory(pc_asctec_sim::pc_traj_cmd * path)
{
	for(int i=0; i<obsNumb; i++) {
		if((obs_ptr+i)->check) {
			path = injectObsPoints(obs_ptr+i, path);
		}
	}
	return path;
}

void updateObstacles(tf::TransformListener * listener)
{
	tf::StampedTransform transform;
	struct obs *ptr = obs_ptr;
	while(ptr->next) {
		listener->lookupTransform(world, ptr->frame, ros::Time(0), transform);
		ptr->x = transform.getOrigin().x();
		ptr->y = transform.getOrigin().y();
		ptr = ptr->next;
	}
}
