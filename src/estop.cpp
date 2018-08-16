#include<ros/ros.h>
#include<suture_knot/EstopAction.h>
#include<actionlib/server/simple_action_server.h>
#include<geometry_msgs/Twist.h>


class EstopAction
{

  public:
    EstopAction(std::string name):
      as_(nh_,name, false),
      action_name_(name)
    {
      as_.registerGoalCallback(boost::bind(&EstopAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&EstopAction::preemptCB, this));
      //add the subscirbers here
      //TODO: modify the pose topic accordingly
	    rob_pos_sub = nh_.subscribe("robot/worldpos", 1,&EstopAction::posCB, this);
      estop_command = false;
      as_.start();
    }
	~EstopAction(void){
	}
    void goalCB()
    {
      ROS_INFO("started the new goal for getting Estop pose");
      estop_command = as_.acceptNewGoal()->get_estop_pos;
    }
    void preemptCB()
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
	void posCB(const geometry_msgs::Twist & _msg){
		if (!as_.isActive())
			return;
		rob_pos = _msg;
		if(!estop_command){
			feedback_.getting_pos= true;
			ROS_INFO("published feedback");
			as_.publishFeedback(feedback_);
		}else{
			result_.estop_pos = rob_pos;
			as_.setSucceeded(result_);
			ROS_INFO("freeze!!!");
		}
	}


    protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<suture_knot::EstopAction> as_;
      std::string action_name_;
	    geometry_msgs::Twist rob_pos;
      suture_knot::EstopFeedback feedback_;
      suture_knot::EstopResult result_;
	    ros::Subscriber rob_pos_sub;
      bool estop_command;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "estop");
  EstopAction estop(ros::this_node::getName());
  ros::spin();

  return 0;
}
