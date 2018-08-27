#include<ros/ros.h>
#include<iiwa_msgs/JointPosition.h>
#include<iiwa_msgs/JointVelocity.h>
#include<iiwa_msgs/JointTorque.h>
#include<sensor_msgs/JointState.h>

#include <string>
#include <sstream>







// read the reference trajectory from the reflexxes node e.g. ref xyz-rpy
iiwa_msgs::JointPosition pos;
iiwa_msgs::JointVelocity vel;
iiwa_msgs::JointTorque torq;

void get_states(const sensor_msgs::JointState & data){
	pos.position.a1 = data.position[0];
    pos.position.a2 = data.position[1];
    pos.position.a3 = data.position[2];
    pos.position.a4 = data.position[3];
    pos.position.a5 = data.position[4];
    pos.position.a6 = data.position[5];
    pos.position.a7 = data.position[6];

	vel.velocity.a1 = data.velocity[0];
    vel.velocity.a2 = data.velocity[1];
    vel.velocity.a3 = data.velocity[2];
    vel.velocity.a4 = data.velocity[3];
    vel.velocity.a5 = data.velocity[4];
    vel.velocity.a6 = data.velocity[5];
    vel.velocity.a7 = data.velocity[6];


	torq.torque.a1 = data.effort[0];
    torq.torque.a2 = data.effort[1];
    torq.torque.a3 = data.effort[2];
    torq.torque.a4 = data.effort[3];
    torq.torque.a5 = data.effort[4];
    torq.torque.a6 = data.effort[5];
    torq.torque.a7 = data.effort[6];


}


int main(int argc, char * argv[]){
	// define the ros node
	ros::init(argc,argv, "sim2rtt");
	ros::NodeHandle nh_;




	
	//ros::NodeHandle home("~");
//	home.getParam("reflexx",reflexx);

	
	// setting up the loop frequency 
	int loop_freq = 100;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	
	// defining the publisher that accepts joint position commands and applies them to the simulator
	ros::Publisher pos_pub = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/state/JointPosition",1);
	ros::Publisher vel_pub = nh_.advertise<iiwa_msgs::JointVelocity>("/iiwa/state/JointVelocity",1);
	ros::Publisher torq_pub = nh_.advertise<iiwa_msgs::JointTorque>("/iiwa/state/JointTorque",1);


	// subscriber for reading the joint angles from the rtt control code
	ros::Subscriber joints_sim_sub = nh_.subscribe("/iiwa/joint_states",1, get_states);


	
 
	




	while(ros::ok()){		
		pos.header.stamp = ros::Time::now();
		vel.header.stamp = ros::Time::now();
		torq.header.stamp = ros::Time::now();
		pos_pub.publish(pos);
		vel_pub.publish(vel);
		torq_pub.publish(torq);

		
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
