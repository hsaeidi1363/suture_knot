#include<ros/ros.h>
#include<suture_knot/FulldrvAction.h>
#include<suture_knot/ReachAction.h>
#include<suture_knot/ForcecheckAction.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Wrench.h>

using namespace suture_knot;

typedef actionlib::SimpleActionClient<suture_knot::ReachAction> reach_client;
typedef actionlib::SimpleActionClient<suture_knot::FulldrvAction> fulldrv_client;
typedef actionlib::SimpleActionClient<suture_knot::ForcecheckAction> forcecheck_client;



void doneReach(const actionlib::SimpleClientGoalState& state,
	           const ReachResultConstPtr& result){
	ROS_INFO("end of reach");
}	

void doneFulldrv(const actionlib::SimpleClientGoalState& state,
	           const FulldrvResultConstPtr& result){
	ROS_INFO("end of fulldrv");
}	

void doneForcecheck(const actionlib::SimpleClientGoalState& state,
	           const ForcecheckResultConstPtr& result){
	ROS_INFO("end of forcecheck");
}	


void activeReach(){
	ROS_INFO("reach is active");
}

void activeFulldrv(){
	ROS_INFO("fulldrv is active");
}

void activeForcecheck(){
	ROS_INFO("forcecheck is active");
}

void feedbackReach(const ReachFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback from reach");
}

void feedbackFulldrv(const FulldrvFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback from fulldrv");
}

void feedbackForcecheck(const ForcecheckFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback from forcecheck");
}



int main (int argc, char **argv){
  ros::init(argc, argv, "knot_client");

  forcecheck_client ac3_("forcecheck", true);
//  actionlib::SimpleActionClient<suture_knot::FulldrvAction> ac2_("fulldrv", true);
  ROS_INFO("waiting for the FORCECHECK action server to start.");
  ac3_.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  suture_knot::ForcecheckGoal goal3;
  goal3.force_limit.force.x = 1.0;
  goal3.force_limit.force.y = 1.0;
  goal3.force_limit.force.z = 1.0;
  goal3.force_limit.torque.x = 1.0;
  goal3.force_limit.torque.y = 1.0;
  goal3.force_limit.torque.z = 1.0;

//  ac2_.sendGoal(goal2);
  ac3_.sendGoal(goal3, &doneForcecheck, &activeForcecheck, &feedbackForcecheck);



  reach_client ac1_("reach", true);
//  actionlib::SimpleActionClient<suture_knot::ReachAction> ac1_("reach", true);
  ROS_INFO("waiting for the REACH action server to start.");
  ac1_.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  suture_knot::ReachGoal goal1;
  //geometry_msgs::Twist start;
  geometry_msgs::Twist target;
  target.linear.x = 0.0100001413817;
  target.linear.y = -0.540000776305;
  target.linear.z = 0.714293314365; 

  target.angular.x = 3.09999794506;
  target.angular.y = 0.00148121916563;
  target.angular.z = 1.84149959732;

  //geometry_msgs::Wrench force_limit;
  //goal1.start = start;
  goal1.target = target;
//  goal1.force_limit = force_limit;
//  ac1_.sendGoal(goal1);
  ac1_.sendGoal(goal1, &doneReach, &activeReach, &feedbackReach);
/*  bool finished_before_timeout1 = ac1_.waitForResult(ros::Duration(15.0));
  if(finished_before_timeout1){  
    actionlib::SimpleClientGoalState state1 = ac1_.getState();
    ROS_INFO("Action finished: %s", state1.toString().c_str());
  }else
    ROS_INFO("Action did not finish before the time out");
*/
  fulldrv_client ac2_("fulldrv", true);
//  actionlib::SimpleActionClient<suture_knot::FulldrvAction> ac2_("fulldrv", true);
  ROS_INFO("waiting for the FULLDRV action server to start.");
  ac2_.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  suture_knot::FulldrvGoal goal2;
  goal2.start_fulldrv = true;
//  ac2_.sendGoal(goal2);
  ac2_.sendGoal(goal2, &doneFulldrv, &activeFulldrv, &feedbackFulldrv);
//  bool finished_before_timeout2 = ac2_.waitForResult(ros::Duration(5.0));
//  if(finished_before_timeout2){  
//    actionlib::SimpleClientGoalState state2 = ac2_.getState();
//    ROS_INFO("Action finished: %s", state2.toString().c_str());
//  }else
//    ROS_INFO("Action did not finish before the time out");



  ros::spin();
  return 0;
}
