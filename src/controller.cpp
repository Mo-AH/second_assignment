#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Speed_service.h"
#include "std_srvs/Empty.h"

//global variables of ranges vector and directions vectors
//to be initialized just once and be used by other functions without passing them
#define RANGES_SIZE 720
#define DIRECTION_SIZE 144 //(720/5)
float ranges[RANGES_SIZE];
float left[DIRECTION_SIZE];
float front[DIRECTION_SIZE];
float right[DIRECTION_SIZE];

//Empty request to reset position
std_srvs::Empty reset;

//global publisher variable to be used by the callback function and the main
ros::Publisher pub;

//distance treshold for front
float front_th = 1.5;

//speed values
float linearSpeed = 0;
float angularSpeedZ = 0;
float angularSpeedX = 0;

int choose_direction() {
/* 	Function to find in what direction is better to go.
	If front is free (every value of front[] > threshold) it choose the front direction.
	Else, it compares the minimum between the left and the right side,
	choosing the direction more distant.

	RETURN:
	direction (int): 0 is left, 1 is front, 2 is right */
 

	//calculates the minimum value for every direction
	float minLeft = 25, minFront = 25, minRight = 25;
	for(int i=0; i<DIRECTION_SIZE; i++){
		if (minLeft > left[i])
			minLeft = left[i];
		if (minFront > front[i])
			minFront = front[i];
		if (minRight > right[i])
			minRight = right[i];
	}

	//if a front wall is near, check what direction between left/right is more far and choose this direction
	if (minFront < front_th){
		if (minLeft > minRight)
			return 0;
		else
			return 2;
	}
	
	//if front is free (less than the threshold), choose the front direction
	return 1;

}

void move(int direction){
/*	Moves the robot in the direction given in input. Depending on it,
	turns left, goes forward or turns right.

	ARGS:
	direction (int): 0 is left, 1 is forward, 2 is right */

	geometry_msgs::Twist vel;

	if (direction == 0){					//LEFT
		vel.angular.z = angularSpeedZ;
		vel.linear.x = angularSpeedX;
	} else if (direction == 1){				//FRONT
		vel.linear.x = linearSpeed;
		vel.angular.z = 0;
	} else if (direction == 2) {			//RIGHT
		vel.angular.z = -angularSpeedZ;
		vel.linear.x = angularSpeedX;
	}

	pub.publish(vel);
}

void control(const sensor_msgs::LaserScan::ConstPtr &msg){
	
	//take the distances's vector
	for (int i = 0; i < RANGES_SIZE; i++) {
		ranges[i] = msg->ranges[i];
	}

	//initialize the subsections vectors
	//take 1st subsection(right), 3rd subsections(front), 5th subsection(left)
	for (int i = 0; i< DIRECTION_SIZE; i++){
		left[i] = ranges[4*DIRECTION_SIZE + i];
		front[i] = ranges[2*DIRECTION_SIZE + i];
		right[i] = ranges[i];
	}

	//move in the direction found by the function choose_direction
	move(choose_direction());

}

bool setSpeed(second_assignment::Speed_service::Request &req, second_assignment::Speed_service::Response &res)
{
    //increase/decrease speed, start moving or stop/reset position
    if (req.input == 'H' || req.input == 'h')
        linearSpeed += 1;
    else if (req.input == 'G' || req.input == 'g')
        linearSpeed -= 1;
	else if (req.input == 'J' || req.input == 'j')
        angularSpeedZ -= 0.2;
    else if (req.input == 'K' || req.input == 'k')
        angularSpeedZ += 0.2;
    else if (req.input == 'R' || req.input == 'r') {
        linearSpeed = 0;
		angularSpeedZ = 0;
        ros::service::call("/reset_positions", reset);
    } else if (req.input == 'S' || req.input == 's') {
		linearSpeed = 0;
		angularSpeedZ = 0;
	} else if (req.input == 'X' || req.input == 'x'){
		linearSpeed = 5;
		angularSpeedZ = 1.3;
	}

	//if statement to control the linear speed of rotation
	if(angularSpeedZ>0)
		if(angularSpeedZ>1.8)
			angularSpeedX = 0.6;
		else if (angularSpeedZ>1)
			angularSpeedX = 0.4;
		else
			angularSpeedX = 0.2;
	else
		angularSpeedX = 0;

	//response to client (return speed values)
	res.linear = linearSpeed;
	res.angular = angularSpeedZ;

    return true;
}

int main(int argc, char ** argv)
{
	//initialize the node
	ros::init(argc, argv, "robot_node");
	ros::NodeHandle nh;

	//create subscriber to /base_scan topic to get info about the walls
	ros::Subscriber subScan = nh.subscribe("/base_scan", 1, control);
	
	//define name of the service and callback
    ros::ServiceServer service = nh.advertiseService("/speed_service", setSpeed);

	//create publisher to /cmd_vel topic to set velocities
	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);

	ros::spin();
	return 0;
}
