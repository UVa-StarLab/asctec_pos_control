#include <asctec_control.h>
#include <string.h>

using namespace std;

AscTecController::GOAL_DATA new_goal;

void goal_callback(const pc_asctec_sim::pc_goal_cmd::ConstPtr& msg)
{
   
   ROS_INFO("New Waypoint Set");

   new_goal.goal_x = msg->x;
   new_goal.goal_y = msg->y;
   new_goal.goal_z = msg->z;
   new_goal.goal_yaw = msg->yaw;
   new_goal.goal_arrival = false;
   new_goal.goal_range = msg->goal_limit;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "pos_controller");
   ros::NodeHandle nh;
   
   //AscTecController asctec_quad;

   AscTecController::PARAM_DATA parameters;
   string global_frame, quad_frame, cmd_topic, sim_topic;

   ros::param::get("~k_p_x", parameters.k_p_x); 
   ros::param::get("~k_i_x", parameters.k_i_x); 
   ros::param::get("~k_d_x", parameters.k_d_x);

   ros::param::get("~k_p_y", parameters.k_p_y); 
   ros::param::get("~k_i_y", parameters.k_i_y); 
   ros::param::get("~k_d_y", parameters.k_d_y);

   ros::param::get("~k_p_z", parameters.k_p_z); 
   ros::param::get("~k_i_z", parameters.k_i_z); 
   ros::param::get("~k_d_z", parameters.k_d_z);   
 
   ros::param::get("~k_p_yaw", parameters.k_p_yaw); 
   ros::param::get("~k_i_yaw", parameters.k_i_yaw); 
   ros::param::get("~k_d_yaw", parameters.k_d_yaw); 

   ros::param::get("~cmd_topic", cmd_topic);
   ros::param::get("~sim_topic", sim_topic);
   ros::param::get("~world_frame", global_frame); 
   ros::param::get("~quad_frame", quad_frame);

   #if REAL
   ros::Publisher accel_quad_cmd = nh.advertise<pc_asctec_sim::SICmd>
                                   (cmd_topic, 10);
   #else
   ros::Publisher accel_sim_cmd = nh.advertise<quadrotor_msgs::PositionCommand>
                                  (sim_topic, 10);
   #endif

   ros::Subscriber pc_goal_cmd;
   ros::Rate rate(CONTROL_RATE);

   tf::StampedTransform transform;
   tf::TransformListener listener;

   //asctec_quad.set_frames(global_frame, quad_frame);
   //asctec_quad.set_parameters(&parameters);

   while (ros::ok()) 
   {
      
      #if REAL
      #else
      #endif
      ros::spinOnce();
      rate.sleep();
   }  
}
