// This action server uses Reflexxes to produce a smooth Cartesian trajectory for the robot to a target point

#include<ros/ros.h>
#include<suture_knot/ReachAction.h>
#include<actionlib/server/simple_action_server.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


// can be changed later
#define CYCLE_TIME_IN_SECONDS                   0.005
// for target XYZ-RPY
#define NUMBER_OF_DOFS                          6


class ReachAction
{
  public:
    ReachAction(std::string name):
      as_(nh_,name, false),
      action_name_(name)
    {
      as_.registerGoalCallback(boost::bind(&ReachAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&ReachAction::preemptCB, this));
      //add the subscirbers here: we are now reading from the simulator to get the initial position of the robot and produce the trajectory for star_rtt updatehook 
  	  rob_pos_sub = nh_.subscribe("robot/worldpos", 1,&ReachAction::posCB, this);
      // publisher for the output of the trajectory generator
  	  reflexxes_pub = nh_.advertise<geometry_msgs::Twist>("/reftraj",1);
	    ResultValue = 0;
	    RML =   new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
      IP  =   new RMLPositionInputParameters(NUMBER_OF_DOFS);
      OP  =   new RMLPositionOutputParameters(NUMBER_OF_DOFS);
      // check if the start and target coordinates are passed to the solver
  	  plan_initialized = false;
      as_.start();
    }
  	~ReachAction(void){
   		delete  RML         ;
	    delete  IP          ;
	    delete  OP          ;
    }
    void goalCB(){
      target_pos = as_.acceptNewGoal()->target;
  	  plan_initialized = false;
      ROS_INFO("received a new target and started the trajectory from the current robot position");
    }
    void preemptCB()
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
    // updates based on the external triggers from the robot pose publisher
  	void posCB(const geometry_msgs::Twist & _msg){
	  	if (!as_.isActive())
		  	return;
	  	rob_pos = _msg;
	  	if(!plan_initialized)
	  		initialize_plan(IP);
	  	else{
		  	 ROS_INFO("working on the plan");
			   if (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED){
			  	// Calling the Reflexxes OTG algorithm
			  	ResultValue =   RML->RMLPosition(*IP, OP, Flags);
			  	ROS_INFO("got a piece of plan");

				  if (ResultValue < 0)
			  	{
				  	ROS_INFO("An error occurred (%i)", ResultValue );
				  	return;
			  	}
				  //do we need to update via the current robot pose?!			
			  	*IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
				  *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
			  	*IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
			  	ref.linear.x = IP->CurrentPositionVector->VecData[0];
			  	ref.linear.y = IP->CurrentPositionVector->VecData[1];
			  	ref.linear.z = IP->CurrentPositionVector->VecData[2];
			  	ref.angular.x = IP->CurrentPositionVector->VecData[3];
			  	ref.angular.y = IP->CurrentPositionVector->VecData[4];
			  	ref.angular.z = IP->CurrentPositionVector->VecData[5];
			  	reflexxes_pub.publish(ref);
				  ROS_INFO("published plan");
				  feedback_.current_waypoint = ref;
			  	ROS_INFO("published feedback");
				  as_.publishFeedback(feedback_);

					
			  }else if (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED){
			  	result_.target_reached = true;
			  	as_.setSucceeded(result_);
			  	ROS_INFO("got to the end!");
				
			  }
		  }
	  }


	void initialize_plan(RMLPositionInputParameters  *_IP){
		
		_IP->CurrentPositionVector->VecData      [0] =    rob_pos.linear.x	;
		_IP->CurrentPositionVector->VecData      [1] =    rob_pos.linear.y   ;
		_IP->CurrentPositionVector->VecData      [2] =    rob_pos.linear.z   ;
		_IP->CurrentPositionVector->VecData      [3] =    rob_pos.angular.x   ;
		_IP->CurrentPositionVector->VecData      [4] =    rob_pos.angular.y   ;
		_IP->CurrentPositionVector->VecData      [5] =    rob_pos.angular.z   ;
		ROS_INFO(" initialized the plan with: %f, %f, %f, %f, %f, %f",rob_pos.linear.x, rob_pos.linear.y, rob_pos.linear.z,rob_pos.angular.x,rob_pos.angular.y,rob_pos.angular.z);
		//setting the target velcoity
		_IP->TargetPositionVector->VecData       [0] =   target_pos.linear.x    ;
		_IP->TargetPositionVector->VecData       [1] =   target_pos.linear.y    ;
		_IP->TargetPositionVector->VecData       [2] =   target_pos.linear.z    ;
		_IP->TargetPositionVector->VecData       [3] =   target_pos.angular.x   ;
		_IP->TargetPositionVector->VecData       [4] =   target_pos.angular.y   ;
		_IP->TargetPositionVector->VecData       [5] =   target_pos.angular.z   ;
		ROS_INFO(" for the target: %f, %f, %f, %f, %f, %f",target_pos.linear.x, target_pos.linear.y, target_pos.linear.z,target_pos.angular.x,target_pos.angular.y,target_pos.angular.z);

		for(int i=0; i < NUMBER_OF_DOFS; i++){
			_IP->CurrentVelocityVector->VecData      [i] =    0.0;
			_IP->CurrentAccelerationVector->VecData  [i] =    0.0;
			_IP->MaxVelocityVector->VecData          [i] =    1.0;
			_IP->MaxAccelerationVector->VecData      [i] =    5.0;
			_IP->MaxJerkVector->VecData              [i] =    10.0;
			//target
			_IP->TargetVelocityVector->VecData       [i] =    0.0;
		}
		

		//determine which Degrees of freedom should be calculated
		_IP->SelectionVector->VecData            [0] =   true        ;
		_IP->SelectionVector->VecData            [1] =   true        ;
		_IP->SelectionVector->VecData            [2] =   true        ;
		_IP->SelectionVector->VecData            [3] =   true        ;
		_IP->SelectionVector->VecData            [4] =   true        ;
		_IP->SelectionVector->VecData            [5] =   true        ;

		plan_initialized = true;
		ResultValue = 0;
		ROS_INFO("initialized the plan");

	}

    protected:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<suture_knot::ReachAction> as_;
      std::string action_name_;
      geometry_msgs::Twist ref;
      geometry_msgs::Twist target_pos;
	    geometry_msgs::Twist rob_pos;
      suture_knot::ReachFeedback feedback_;
      suture_knot::ReachResult result_;
	    ros::Subscriber rob_pos_sub;
  	  bool plan_initialized;
	    ros::Publisher reflexxes_pub;
	    int                         ResultValue ;
      ReflexxesAPI                *RML                        =   NULL    ;
      RMLPositionInputParameters  *IP                         =   NULL    ;
      RMLPositionOutputParameters *OP                         =   NULL    ;
      RMLPositionFlags            Flags                                   ;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "reach");
  ReachAction reach(ros::this_node::getName());
  ros::spin();

  return 0;
}
