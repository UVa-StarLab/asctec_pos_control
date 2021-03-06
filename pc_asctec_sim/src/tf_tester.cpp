#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "tf_broadcaster_test");
	ros::NodeHandle nh;

	std::string q_frame, t_frame;
	ros::param::get("~q_frame", q_frame);
	ros::param::get("~t_frame", t_frame);
	ros::Rate rate(60.0);

	static tf::TransformBroadcaster br;
	tf::Transform transform, tiki;
	transform.setOrigin(tf::Vector3(0.0, -2.0, 1.0));
	tiki.setOrigin(tf::Vector3(0, 0.65, 1.4));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	tiki.setRotation(q);
	tf::TransformListener listener;

	bool temp = listener.waitForTransform("odom", q_frame, ros::Time(0), ros::Duration(0.5));
	if(temp) {
		ros::shutdown();
	}else {
		if(q_frame != "") {
			ROS_INFO_STREAM("Setting up testing frame: " + q_frame);
		}
		if(t_frame != "") {
			ROS_INFO_STREAM("Setting up testing frame: " + t_frame);
		}
	}

	while(ros::ok()) {
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", q_frame));
		br.sendTransform(tf::StampedTransform(tiki, ros::Time::now(), "odom", t_frame));
		rate.sleep();
	}

	return 0;
};


