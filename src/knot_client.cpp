#include<ros/ros.h>
#include<suture_knot/FulldrvAction.h>
#include<suture_knot/ReachAction.h>
#include<suture_knot/ForcecheckAction.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Wrench.h>


using namespace suture_knot;

//also need reading the current rob pose and caluclate the traj
// read some commands for starting the suturing from a GUI
class KnotClient{
	public:
		ros::NodeHandle nh_;
		typedef actionlib::SimpleActionClient<suture_knot::ReachAction> reach_client;
		typedef actionlib::SimpleActionClient<suture_knot::FulldrvAction> fulldrv_client;
		typedef actionlib::SimpleActionClient<suture_knot::ForcecheckAction> forcecheck_client;
		reach_client ac1_;//("reach", true);
		fulldrv_client ac2_;//("fulldrv", true);
		forcecheck_client ac3_;//("forcecheck", true);
		// constructor  
		KnotClient(ros::NodeHandle nh):
  			nh_( nh ),
			ac1_(nh, "reach",true),
			ac2_(nh,"fulldrv", true),
			ac3_(nh,"forcecheck",true)			
			{	
			ROS_INFO("enterened the constructor");	
			
			ROS_INFO("waiting for the REACH action server to start.");
			ac1_.waitForServer();
			ROS_INFO("REACH Action server started.");

			ROS_INFO("waiting for the FULLDRV action server to start.");
			ac2_.waitForServer();
			ROS_INFO("FULLDRV Action server started.");


			ROS_INFO("waiting for the FORCECHECK action server to start.");
			ac3_.waitForServer();
			ROS_INFO("FORCECHECK Action server started.");
		}
		//Destructor
		~KnotClient(){
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

	private:
		enum state {BITE, FIRESTICH, STRETCH, TURN};
};







int main (int argc, char **argv){
	ros::init(argc, argv, "knot_client");
	ros::NodeHandle nh;

	KnotClient knot_client(nh);
  

	suture_knot::ForcecheckGoal goal3;
	goal3.force_limit.force.x = 1.0;
	goal3.force_limit.force.y = 1.0;
	goal3.force_limit.force.z = 1.0;
	goal3.force_limit.torque.x = 1.0;
	goal3.force_limit.torque.y = 1.0;
	goal3.force_limit.torque.z = 1.0;

	knot_client.executeForcecheck(goal3);				


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




	suture_knot::FulldrvGoal goal2;
	goal2.start_fulldrv = true;
	knot_client.executeFulldrv(goal2);				



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
