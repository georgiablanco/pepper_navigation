/*
 *  Gaitech Educational Portal
 *
 * Copyright (c) 2016
 * All rights reserved.
 *
 *   Program: Map-Based Navigation
 */

 /***************** EDITED BY GEORGIA BLANCO-LITCHFIELD *****************/
 /*********** FOR USE WITH PEPPER_NAVGIATION PACKAGE AND MAPS ***********/


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal, double oGoal, double owGoal);
char choose();

/** Restaurant poses and orientations of interest **/
//Restaurant 1
double xRoom1A = -3.16;
double yRoom1A = 2.36;
double ozRoom1A = 0.387;
double owRoom1A = 0.55;
double xRoom1B = 2.57;
double yRoom1B = -3.57;
double ozRoom1B = -1.00;
double owRoom1B = 0.00;
//Restaurant 2
double xRoom2A = -0.92;
double yRoom2A = 6.67;
double ozRoom2A = -1.00;
double owRoom2A = 0.01;
double xRoom2B = -3.37;
double yRoom2B = -1.29;
double owRoom2B = 1.00;
double ozRoom2B = 0.00;
//Restaurant 3
double xRoom3A = -1.02;
double yRoom3A = 7.87;
double ozRoom3A = -0.02;
double owRoom3A = 0.96;
double xRoom3B = 1.47;
double yRoom3B = 1.605;
double ozRoom3B = -0.7;
double owRoom3B = 0.68;
//Restaurant 4
double xRoom4A = -2.59;
double yRoom4A = 7.76;
double ozRoom4A = -1.00;
double owRoom4A = 0.18;
double xRoom4B = 4.386;
double yRoom4B = 0.756;
double owRoom4B = 0.84;
double ozRoom4B = 0.53;


bool goalReached = false;

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();

	//tell the action client that we want to spin a thread by default

	char choice = 'q';
	do{
		choice = choose();
		if (choice == '0'){
			goalReached = moveToGoal(-2.437, -5.333, 0.00, 1.00);
		}else if (choice == '1'){
			goalReached = moveToGoal(xRoom1A, yRoom1A, ozRoom1A, owRoom1A);
		}else if (choice == '2'){
			goalReached = moveToGoal(xRoom1B, yRoom1B, ozRoom1B, ozRoom1B);
		}else if (choice == '3'){
			goalReached = moveToGoal(xRoom2A, yRoom2A, ozRoom2A, owRoom2A);
		}else if (choice == '4'){
			goalReached = moveToGoal(xRoom2B, yRoom2B, ozRoom2B, owRoom2B);
		}else if (choice == '5'){
			goalReached = moveToGoal(xRoom3A, yRoom3A, ozRoom3A, owRoom3A);
		}else if (choice == '6'){
			goalReached = moveToGoal(xRoom3B, yRoom3B, ozRoom3B, owRoom3B);
		}else if (choice == '7'){
			goalReached = moveToGoal(xRoom4A, yRoom4A, ozRoom4A, owRoom4A);
		}else if (choice == '8'){
			goalReached = moveToGoal(xRoom4B, yRoom4B, ozRoom4B, owRoom4B);
		}
		if (choice!='q'){
			if (goalReached){
				ROS_INFO("Congratulations!");
				ros::spinOnce();


			}else{
				ROS_INFO("Hard Luck!");
			}
		}
	}while(choice !='q');
	return 0;
}

bool moveToGoal(double xGoal, double yGoal, double ozGoal, double owGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = ozGoal;
	goal.target_pose.pose.orientation.w = owGoal;


	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);
	time_t start = time(0);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	    float execution_time = time(0) - start;
		ROS_INFO("You have reached the destination in %f seconds!", execution_time);
		return true;
	}
	else{
	    float execution_time = time(0) - start;
		ROS_INFO("The robot failed to reach the destination in %f seconds!", execution_time);
		return false;
	}

}

char choose(){
	char choice='q';
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|PRESS A KEY TO CHOOSE RESTAURANT & GOAL:"<<std::endl;
	std::cout<<"|'0': Return to Home"<<std::endl;
	std::cout<<"|'1': Restaurant 1 Goal A"<<std::endl;
	std::cout<<"|'2': Restaurant 1 Goal B"<<std::endl;
	std::cout<<"|'3': Restaurant 2 Goal A"<<std::endl;
	std::cout<<"|'4': Restaurant 2 Goal B"<<std::endl;
	std::cout<<"|'5': Restaurant 3 Goal A"<<std::endl;
	std::cout<<"|'6': Restaurant 3 Goal B"<<std::endl;
	std::cout<<"|'7': Restaurant 4 Goal A"<<std::endl;
	std::cout<<"|'8': Restaurant 4 Goal B"<<std::endl;
	std::cout<<"|'q': Quit "<<std::endl;
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|WHERE TO GO?";
	std::cin>>choice;

	return choice;


}