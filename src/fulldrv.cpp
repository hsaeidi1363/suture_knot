#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<actionlib/server/simple_action_server.h>
#include<suture_knot/FulldrvAction.h>
  
class FulldrvAction{
  protected:
    //following ROS format for defining action server
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<suture_knot::FulldrvAction> as_;
    std::string action_name_;
    suture_knot::FulldrvFeedback feedback_;
    suture_knot::FulldrvResult result_;
    // a publisher for the topic that accepts fulldrv commands in the endo360_rtt component
    ros::Publisher fulldrv_pub;
	public:
		FulldrvAction(std::string name):
	    as_(nh_, name,boost::bind(&FulldrvAction::executeCB, this, _1), false),
      action_name_(name)
    {
        // a publisher for the topic that accepts fulldrv commands in the endo360_rtt component
          fulldrv_pub = nh_.advertise<sensor_msgs::JointState>("/endo360/CmdJointState",1);
          // start the action server
      		as_.start();
    }
    ~FulldrvAction(void)
    {
    }
    // once a goal (here "true" value is received send the full drive command wait for it to be performed 
    void executeCB(const suture_knot::FulldrvGoalConstPtr &goal){
      if(goal->start_fulldrv){
        ros::Rate loop_rate(1); //TODO: change it to propper wait time on the actual tool  
        feedback_.firing_suture = true;
        ROS_INFO("Firing a suture (full drive mode)");
      
        sensor_msgs::JointState fulldrv_msg;
        // packing the message components based on the endo360-fulldrv.sh
        fulldrv_msg.header.stamp = ros::Time::now();  
        fulldrv_msg.header.frame_id ="endo360";
        fulldrv_msg.name.push_back("fulldrive");
        fulldrv_msg.name.push_back("inactive");
        fulldrv_msg.name.push_back("inactive");
        fulldrv_msg.name.push_back("inactive");
        fulldrv_msg.name.push_back("inactive");
        fulldrv_msg.name.push_back("position");
        for (int i = 0; i < 6; i++){
          fulldrv_msg.position.push_back(0);
          fulldrv_msg.velocity.push_back(0);
          fulldrv_msg.effort.push_back(0);
        }
        fulldrv_pub.publish(fulldrv_msg);
        int ctr = 0;
        // do we need any preempts?
        while(ros::ok()){
          ctr++;
          if(ctr > 1){
            break;
          }else{
            as_.publishFeedback(feedback_);
          }
          loop_rate.sleep(); 
        }
        result_.fulldrv_done = true;
        ROS_INFO("%s: finished the suture full drive", action_name_.c_str());
        as_.setSucceeded(result_);
      }else{
        ROS_INFO("send a goal with True value to start the full drive");
      }
      
    }

};
int main(int argc, char **argv){
	ros::init(argc, argv, "fulldrv");
  
  FulldrvAction fulldrv("fulldrv");
  ros::spin();		

	return 0;
}
