#include <asctec_quad.h>
#include "asctec_control.cpp"
#include "trajectory_accel.cpp"
#include "asctec_quad.cpp"

typedef enum states {wait, traj} st_;
states state = wait;
QUAD_CMD cmd;
QUAD_OUT *feedback = new QUAD_OUT;

bool timing = false;
ros::Timer timer_event;
visualization_msgs::Marker traj_trail, quad_trail;
tf::StampedTransform qtransform;

void configCallback(const pc_asctec_sim::ascTunerConfig &config, uint32_t level) 
{
	cmd.kvals.kpx = config.kpx;
	cmd.kvals.kix = config.kix;
	cmd.kvals.kvx = config.kvx;
	cmd.kvals.kax = config.kax;

	cmd.kvals.kpy = config.kpy;
	cmd.kvals.kiy = config.kiy;
	cmd.kvals.kvy = config.kvy;
	cmd.kvals.kay = config.kay;

	cmd.kvals.kpz = config.kpz;
	cmd.kvals.kiz = config.kiz;
	cmd.kvals.kvz = config.kvz;
	cmd.kvals.kaz = config.kaz;

	cmd.kvals.kpyaw = config.kpyaw;
	cmd.kvals.kiyaw = config.kiyaw;
	cmd.kvals.kvyaw = config.kvyaw;
	cmd.kvals.kayaw = config.kayaw;
}

void llCallback(const pc_asctec_sim::LLStatus::ConstPtr& msg)
{
	cmd.battery = msg->battery_voltage;
}

void timerCallback(const ros::TimerEvent&) {
	timing = false;
	cmd.stopTimer = true;
	timer_event.stop();
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

}
void pathCallback(const pc_asctec_sim::pc_traj_cmd::ConstPtr& msg) 
{
	if(msg->points != 0) {
		cmd.f_path = *msg;
		cmd.newPath = true;
	}else {
		ROS_INFO("Set number of points; path ignored");
	}
}

void initTrails(string frame)
{
	traj_trail.header.frame_id = frame;
	traj_trail.header.stamp = ros::Time::now();
	traj_trail.id = 2;
	traj_trail.action = visualization_msgs::Marker::ADD;
	traj_trail.type = visualization_msgs::Marker::LINE_LIST;
	traj_trail.color.a = 1.0;
	traj_trail.color.g = 1.0;				
	traj_trail.color.b = 1.0;
	traj_trail.scale.x = 0.05;
	traj_trail.scale.y = 0.05;

	quad_trail.header.frame_id = frame;
	quad_trail.header.stamp = ros::Time::now();
	quad_trail.id = 2;
	quad_trail.action = visualization_msgs::Marker::ADD;
	quad_trail.type = visualization_msgs::Marker::LINE_LIST;
	quad_trail.color.a = 1.0;				
	quad_trail.color.b = 1.0;
	quad_trail.color.g = 0.7;
	quad_trail.scale.x = 0.05;
	quad_trail.scale.y = 0.05;

	geometry_msgs::Point vis_trail;
	vis_trail.x = qtransform.getOrigin().x();
	vis_trail.y = qtransform.getOrigin().y();
	vis_trail.z = qtransform.getOrigin().z();

	traj_trail.points.push_back(vis_trail);
	quad_trail.points.push_back(vis_trail);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_");
	ros::NodeHandle nh;
	ros::Rate rate(CONTROL_RATE);

	dynamic_reconfigure::Server<pc_asctec_sim::ascTunerConfig> server;
	dynamic_reconfigure::Server<pc_asctec_sim::ascTunerConfig>::CallbackType f;
	f = boost::bind(&configCallback, _1, _2);
	server.setCallback(f);

	timer_event = nh.createTimer(ros::Duration(3.0), timerCallback, true);
	timer_event.stop();

	K_DATA k_vals;
	string q_name, w_frame, q_frame;

	ros::param::get("~w_frame", w_frame);
	ros::param::get("~q_name", q_name);
	ros::param::get("~q_frame", q_frame);

	ros::param::get("~kpx", k_vals.kpx); 
	ros::param::get("~kix", k_vals.kix); 
	ros::param::get("~kvx", k_vals.kvx);
	ros::param::get("~kax", k_vals.kax);

	ros::param::get("~kpy", k_vals.kpy); 
	ros::param::get("~kiy", k_vals.kiy); 
	ros::param::get("~kvy", k_vals.kvy);
	ros::param::get("~kay", k_vals.kay);

	ros::param::get("~kpz", k_vals.kpz); 
	ros::param::get("~kiz", k_vals.kiz); 
	ros::param::get("~kvz", k_vals.kvz);	
	ros::param::get("~kaz", k_vals.kaz);
 
	ros::param::get("~kpyaw", k_vals.kpyaw); 
	ros::param::get("~kiyaw", k_vals.kiyaw); 
	ros::param::get("~kvyaw", k_vals.kvyaw);
	ros::param::get("~kayaw", k_vals.kayaw);

	ros::Publisher end_pub = nh.advertise<std_msgs::Empty>(q_name + "/traj_end",10);
	ros::Publisher tTrail_pub = nh.advertise<visualization_msgs::Marker>(q_name + "/trajectory_trail",10);
	ros::Publisher viz_pub = nh.advertise<geometry_msgs::PointStamped>(q_name + "/viz_goals",10);
	ros::Publisher cmd_pub = nh.advertise<pc_asctec_sim::SICmd>(q_name + "/cmd_si", 10);
	ros::Publisher qTrail_pub = nh.advertise<visualization_msgs::Marker>(q_name + "/quad_trail",10);

	ros::Subscriber path_sub = nh.subscribe(q_name + "/traj_points", 1, pathCallback);
	ros::Subscriber joy_sub = nh.subscribe("/joy", 10, joyCallback);
	ros::Subscriber ll_sub = nh.subscribe(q_name + "/ll_status", 1, llCallback);

	tf::TransformListener listener;	
	listener.waitForTransform(w_frame, q_frame, ros::Time(0), ros::Duration(3.0));

	AscTec_Quad asc(q_frame, w_frame, dT, &k_vals);

	ROS_INFO("Asctec Quad Running!");

	while(ros::ok()) {
		ros::spinOnce();

		listener.lookupTransform(w_frame, q_frame, ros::Time(0), qtransform);
		feedback = asc.runQuad(&cmd, &qtransform);
		cmd.newPath = false;
		if(!timing && feedback->startTimer) {
			timing = true;
			timer_event.setPeriod(ros::Duration(feedback->time, true));
			timer_event.start();
		}
		rate.sleep();
	}
	return 0;
}
