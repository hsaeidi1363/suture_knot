#include<ros/ros.h>
#include<suture_knot/FulldrvAction.h>
#include<suture_knot/ReachAction.h>
#include<suture_knot/ForcecheckAction.h>
#include<suture_knot/EstopAction.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Wrench.h>
#include<std_msgs/Bool.h>

using namespace suture_knot;

//also need reading the current rob pose and caluclate the traj
// read some commands for starting the suturing from a GUI
class KnotClient{
	public:
		ros::NodeHandle nh_;
		typedef actionlib::SimpleActionClient<suture_knot::ReachAction> reach_client;
		typedef actionlib::SimpleActionClient<suture_knot::FulldrvAction> fulldrv_client;
		typedef actionlib::SimpleActionClient<suture_knot::ForcecheckAction> forcecheck_client;
		typedef actionlib::SimpleActionClient<suture_knot::EstopAction> estop_client;
		reach_client ac1_;//("reach", true);
		fulldrv_client ac2_;//("fulldrv", true);
		forcecheck_client ac3_;//("forcecheck", true);
    estop_client ac4_;
		// constructor  
		KnotClient(ros::NodeHandle nh):
  		nh_( nh ),
      suture_started(false),
      reach_started(false),
			ac1_(nh, "reach",true),
			ac2_(nh,"fulldrv", true),
			ac3_(nh,"forcecheck",true),			
			ac4_(nh,"estop",true)			
			{	
		  dbg_poscommand_sub = nh.subscribe("reach_command_dbg",1,&KnotClient::pos_command_callback, this);	
		  dbg_startsuture_sub = nh.subscribe("start_suture_dbg",1,&KnotClient::start_suture_callback, this);	
      forcecheck_goal.force_limit.force.x = 1.0;
      forcecheck_goal.force_limit.force.y = 1.0;
      forcecheck_goal.force_limit.force.z = 1.0;
      forcecheck_goal.force_limit.torque.x = 1.0;
      forcecheck_goal.force_limit.torque.y = 1.0;
      forcecheck_goal.force_limit.torque.z = 1.0;
			ROS_INFO("waiting for the REACH action server to start.");
			ac1_.waitForServer();
			ROS_INFO("REACH Action server started.");

			ROS_INFO("waiting for the FULLDRV action server to start.");
			ac2_.waitForServer();
			ROS_INFO("FULLDRV Action server started.");


			ROS_INFO("waiting for the FORCECHECK action server to start.");
			ac3_.waitForServer();
			ROS_INFO("FORCECHECK Action server started.");

			ROS_INFO("waiting for the ESTOP action server to start.");
			ac4_.waitForServer();
			ROS_INFO("ESTOP Action server started.");
		}
		//Destructor
		~KnotClient(){
		}
    void start_suture_callback(const std_msgs::Bool & _start_command){
      suture_started = _start_command.data;
    }
 
    void pos_command_callback(const geometry_msgs::Twist & _command){
	    reach_goal.target = _command;
      if(suture_started && !reach_started){
      	KnotClient::executeReach(reach_goal);
        reach_started = true;
      	KnotClient::executeForcecheck(forcecheck_goal);
        ROS_INFO("new goal sent to REACH action");				
      }				
    }

		void executeReach(suture_knot::ReachGoal &goal){
			ac1_.sendGoal(goal, boost::bind(&KnotClient::doneReach,this, _1,_2),boost::bind(&KnotClient::activeReach,this),boost::bind(&KnotClient::feedbackReach,this, _1));
		}

		void executeFulldrv(suture_knot::FulldrvGoal &goal){
			ac2_.sendGoal(goal, boost::bind(&KnotClient::doneFulldrv,this, _1,_2),boost::bind(&KnotClient::activeFulldrv,this),boost::bind(&KnotClient::feedbackFulldrv,this, _1));
		}


		void executeForcecheck(suture_knot::ForcecheckGoal &goal){
			ac3_.sendGoal(goal, boost::bind(&KnotClient::doneForcecheck,this, _1,_2),boost::bind(&KnotClient::activeForcecheck,this),boost::bind(&KnotClient::feedbackForcecheck,this, _1));
		}

		void executeEstop(suture_knot::EstopGoal &goal){
			ac4_.sendGoal(goal, boost::bind(&KnotClient::doneEstop,this, _1,_2),boost::bind(&KnotClient::activeEstop,this),boost::bind(&KnotClient::feedbackEstop,this, _1));
		}


		void doneReach(const actionlib::SimpleClientGoalState& state,
					   const ReachResultConstPtr& result){
			ROS_INFO("end of reach");
      reach_started = false;
    	fulldrv_goal.start_fulldrv = true;
    	KnotClient::executeFulldrv(fulldrv_goal);				
		}	

		void doneFulldrv(const actionlib::SimpleClientGoalState& state,
					   const FulldrvResultConstPtr& result){
			ROS_INFO("end of fulldrv");
		}	

		void doneForcecheck(const actionlib::SimpleClientGoalState& state,
					   const ForcecheckResultConstPtr& result){
			ROS_INFO("forcecheck detected a force limit violation");
      if (result->force_limit_reached){
        estop_goal.get_estop_pos = true;
        KnotClient::executeEstop(estop_goal); 
        ROS_INFO("checking current pose for ESTOP");
      }
		}	

		void doneEstop(const actionlib::SimpleClientGoalState& state,
					   const EstopResultConstPtr& result){
			ROS_INFO("end of estop");
      reach_goal.target = result->estop_pos;
    	KnotClient::executeReach(reach_goal);				
      ROS_INFO("ESTOP command sent");
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

		void activeEstop(){
			ROS_INFO("estop is active");
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

		void feedbackEstop(const EstopFeedbackConstPtr& feedback)
		{
		  ROS_INFO("Got Feedback from estop");
		}
	private:
		enum state {BITE, FIRESUTURE, STRETCH, TURN};
    ros::Subscriber dbg_poscommand_sub;
    ros::Subscriber dbg_startsuture_sub; 
    suture_knot::ReachGoal reach_goal;
    suture_knot::ForcecheckGoal forcecheck_goal;
    suture_knot::EstopGoal estop_goal;
    suture_knot::FulldrvGoal fulldrv_goal;

    bool suture_started;     
    bool reach_started;
};

int main (int argc, char **argv){
	ros::init(argc, argv, "knot_client");
	ros::NodeHandle nh;

	KnotClient knot_client(nh);
  

/*

	suture_knot::ReachGoal goal1;
	geometry_msgs::Twist target;
	target.linear.x = 0.0100001413817;
	target.linear.y = -0.540000776305;
	target.linear.z = 0.714293314365; 
	target.angular.x = 3.09999794506;
	target.angular.y = 0.00148121916563;
	target.angular.z = 1.84149959732;
	goal1.target = target;
	knot_client.executeReach(goal1);				
*/

//	suture_knot::FulldrvGoal goal2;



  ros::spin();
  return 0;
}

		//  actionlib::SimpleActionClient<suture_knot::FulldrvAction> ac2_("fulldrv", true);
		//  actionlib::SimpleActionClient<suture_knot::ReachAction> ac1_("reach", true);
		//  actionlib::SimpleActionClient<suture_knot::FulldrvAction> ac2_("fulldrv", true);

/*  bool finished_before_timeout1 = ac1_.waitForResult(ros::Duration(15.0));
					  if(finished_before_timeout1){  
						actionlib::SimpleClientGoalState state1 = ac1_.getState();
						ROS_INFO("Action finished: %s", state1.toString().c_str());
					  }else
						ROS_INFO("Action did not finish before the time out");
					*/
