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
		bool exessive_fx = ForcecheckAction::limit_reached(rob_force.force.x, force_limit.force.x); 
		bool exessive_fy = ForcecheckAction::limit_reached(rob_force.force.y, force_limit.force.y); 
		bool exessive_fz = ForcecheckAction::limit_reached(rob_force.force.z, force_limit.force.z); 
		bool exessive_tx = ForcecheckAction::limit_reached(rob_force.torque.x, force_limit.torque.x); 
		bool exessive_ty = ForcecheckAction::limit_reached(rob_force.torque.y, force_limit.torque.y); 
		bool exessive_tz = ForcecheckAction::limit_reached(rob_force.torque.z, force_limit.torque.z); 
		bool exessive_force = exessive_fx || exessive_fy ||exessive_fz || exessive_tx || exessive_ty || exessive_tz;
		if(!exessive_force){
			feedback_.current_force = rob_force;
			ROS_INFO("published feedback");
			as_.publishFeedback(feedback_);
		}else{
			result_.force_limit_reached = true;
			as_.setSucceeded(result_);
			ROS_INFO("don't break the tissue!!!");
		}
	}


  bool limit_reached(float _in, float _limit){
    if ( (_limit < 0 && _in <= _limit) || (_limit > 0 && _in >= _limit) )
      return true;
    else 
      return false;  
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
