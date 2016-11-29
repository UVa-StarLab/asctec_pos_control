#include <ros/ros.h>
   
#include <dynamic_reconfigure/server.h>
#include <pc_asctec_sim/tunerConfig.h>
 
void callback(pc_asctec_sim::tunerConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f %f", 
	config.k_p_xy, config.k_i_xy, 
	config.k_d_xy, 
	config.k_a_xy);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "param_tuner");

	dynamic_reconfigure::Server<pc_asctec_sim::tunerConfig> server;
	dynamic_reconfigure::Server<pc_asctec_sim::tunerConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}
