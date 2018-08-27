#include<ros/ros.h>
#include<suture_knot/TurntoolAction.h>
#include<actionlib/server/simple_action_server.h>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float32.h>

class TurntoolAction
{

  public:
    TurntoolAction(std::string name):
      as_(nh_,name, false),
      action_name_(name)
    {
      as_.registerGoalCallback(boost::bind(&TurntoolAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&TurntoolAction::preemptCB, this));
      //here only a dummy topic for the subscriber is defined to check the code logic=> TODO: later change the topic to the actual sensor topic
  	  joints_sub = nh_.subscribe("iiwa/joint_states", 1, &TurntoolAction::jointCB, this);
      as_.start();
    }
	  ~TurntoolAction(void){
  	}
    void goalCB()
    {
      turn_angle = as_.acceptNewGoal()->turn;
      ROS_INFO("Received and accepted a tool turn target");
    }
    void preemptCB()
    {
      ROS_INFO("%s: Preempted (Turn goal)", action_name_.c_str());
      as_.setPreempted();
    }
	//TODO: should be stamped message with the actual sensor
  	void jointCB(const sensor_msgs::JointState & _msg){
	  	if (!as_.isActive())
		  	return;
      // read the current force/torque values from the sensor
   		rob_joints = _msg;
		float current_tool_angle = rob_joints.position[6];
		float desired_tool_angle = current_tool_angle + turn_angle;
	  	if(!exessive_force){
		  	feedback_.finding_coordinates = true;
		  	as_.publishFeedback(feedback_);
  		}else{
	  		result_.force_limit_reached = true;
		  	as_.setSucceeded(result_);
			  ROS_INFO("Watch out: don't break the tissue!!!");
		  }
  	}


  bool calc_target_coordinates(float _in, float _limit){
    if ( (_limit < 0 && _in <= _limit) || (_limit > 0 && _in >= _limit) )
      return true;
    else 
      return false;  
  }
    protected:
      // follow some standard definitions for the action server
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<suture_knot::TurntoolAction> as_;
      std::string action_name_;
	  std_msgs::Float32 turn_angle;
	  sensor_msgs::JointState rob_joints;
      suture_knot::TurntoolFeedback feedback_;
      suture_knot::TurntoolResult result_;
	  ros::Subscriber joints_sub;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "forcecheck");
  ForcecheckAction forcecheck(ros::this_node::getName());

  ros::spin();
  return 0;
}
