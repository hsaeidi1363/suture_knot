#include<ros/ros.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<geometry_msgs/Twist.h>
#include<iiwa_msgs/JointPosition.h>
#include<sensor_msgs/JointState.h>
#include<kdl/chain.hpp>
#include <string>
#include <sstream>



// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_pt.positions.push_back(_init);
}

//defines the joint names for the robot (used in the jointTrajectory messages)
void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj){
	for (int i = 1; i <= _nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "iiwa_joint_";
		joint_name << i;
		_cmd.joint_names.push_back(joint_name.str());
	}
}



// read the reference trajectory from the reflexxes node e.g. ref xyz-rpy
bool ref_received= false;
iiwa_msgs::JointPosition ref;
void get_ref(const iiwa_msgs::JointPosition & data){
	ref = data;
	ref_received = true;
}


int main(int argc, char * argv[]){
	
	// get the number of joints from the chain
	unsigned int nj = 7;
	
	

	// define the ros node
	ros::init(argc,argv, "rtt2sim");
	ros::NodeHandle nh_;

	trajectory_msgs::JointTrajectory joint_cmd;
	trajectory_msgs::JointTrajectoryPoint pt;

	initialize_points(pt,nj,0.0);


	
	//ros::NodeHandle home("~");
//	home.getParam("reflexx",reflexx);

	
	// setting up the loop frequency 
	int loop_freq = 100;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	
	// defining the publisher that accepts joint position commands and applies them to the simulator
	std::string command_topic = "iiwa/PositionJointInterface_trajectory_controller/command";
	ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,1);

	// subscriber for reading the joint angles from the rtt control code
	ros::Subscriber rtt_cmd_sub = nh_.subscribe("/iiwa/command/JointPosition",10, get_ref);
	// subscriber for reading the reference trajectories from the reflexxes-based node	
	ros::Subscriber ref_sub = nh_.subscribe("/reftraj",10, get_ref);

	
 
	// define the joint names, e.g. iiwa_joint_1 up to iiwa_joint_7
	name_joints(joint_cmd, nj);
	



	pt.positions[0]= 0.0;
	pt.positions[1]= 0.0;
	pt.positions[2]= 0.0;
	pt.positions[3]= 0.0;
	pt.positions[4]= 0.0;
	pt.positions[5]= 0.0;
	pt.positions[6]= 0.0;
	pt.time_from_start = ros::Duration(dt);



	joint_cmd.points.push_back(pt);
	//cmd.points.push_back(pt2);

	while(ros::ok()){		
		if (ref_received){
			// update the joint positions with the most recent readings from the joints
			
			pt.positions[0] = ref.position.a1;				
			pt.positions[1] = ref.position.a2;
			pt.positions[2] = ref.position.a3;
			pt.positions[3] = ref.position.a4;
			pt.positions[4] = ref.position.a5;
			pt.positions[5] = ref.position.a6;
			pt.positions[6] = ref.position.a7;

		
			pt.time_from_start = ros::Duration(dt);
			joint_cmd.points[0] = pt;
		
			joint_cmd.header.stamp = ros::Time::now();
			cmd_pub.publish(joint_cmd);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
