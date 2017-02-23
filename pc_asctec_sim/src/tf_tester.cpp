#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "tf_broadcaster_test");
	ros::NodeHandle nh;

	std::string q_frame;
	ros::param::get("~q_frame", q_frame);
	ros::Rate rate(60.0);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.1));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	tf::TransformListener listener;


	bool temp = listener.waitForTransform("odom", q_frame, ros::Time(0), ros::Duration(0.5));
	if(temp) {
		ros::shutdown();
	}else {
		ROS_INFO_STREAM("Setting up testing frame: " + q_frame);
	}

	while(ros::ok()) {
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", q_frame));
		rate.sleep();
	}

	return 0;
};


