#include<ros/ros.h>
#include<suture_knot/ForcecheckAction.h>
#include<actionlib/server/simple_action_server.h>
#include<geometry_msgs/Wrench.h>


class ForcecheckAction
{

  public:
    ForcecheckAction(std::string name):
      as_(nh_,name, false),
      action_name_(name)
    {
      as_.registerGoalCallback(boost::bind(&ForcecheckAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&ForcecheckAction::preemptCB, this));
      //add the subscirbers here
	  force_sub = nh_.subscribe("ATI/force", 1,&ForcecheckAction::forceCB, this);
      as_.start();
    }
	~ForcecheckAction(void){
	}
    void goalCB()
    {
      ROS_INFO("started the new goal for force limit");
      force_limit = as_.acceptNewGoal()->force_limit;
      ROS_INFO("obtained the target force");
    }
    void preemptCB()
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
	//TODO: !!!!!!!!should be stamped message with the actual sensor
	void forceCB(const geometry_msgs::Wrench & _msg){
		if (!as_.isActive())
			return;
		rob_force = _msg;
		float df_x = fabs(force_limit.force.x - rob_force.force.x); 
		float df_y = fabs(force_limit.force.y - rob_force.force.y); 
		float df_z = fabs(force_limit.force.z - rob_force.force.z); 
		float dT_x = fabs(force_limit.torque.x - rob_force.torque.x); 
		float dT_y = fabs(force_limit.torque.y - rob_force.torque.y); 
		float dT_z = fabs(force_limit.torque.z - rob_force.torque.z); 
		float th_f = 0.01; //threshold for force difference between the sensor reading and the limit
		float th_T = 0.01; //threshold for force difference between the sensor reading and the limit
		bool force_reached = df_x < th_f || df_y < th_f || df_z < th_f || dT_x < th_T || dT_y < th_T || dT_z < th_T;
		if(!force_reached){
			feedback_.current_force = rob_force;
			ROS_INFO("published feedback");
			as_.publishFeedback(feedback_);
		}else{
			result_.force_limit_reached = true;
			as_.setSucceeded(result_);
			ROS_INFO("don't break the tissue!!!");
		}
	}



    protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<suture_knot::ForcecheckAction> as_;
      std::string action_name_;
	  geometry_msgs::Wrench rob_force;
	  geometry_msgs::Wrench force_limit;
      suture_knot::ForcecheckFeedback feedback_;
      suture_knot::ForcecheckResult result_;
	  ros::Subscriber force_sub;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "forcecheck");
  ForcecheckAction forcecheck(ros::this_node::getName());
  ros::spin();

  return 0;
}
