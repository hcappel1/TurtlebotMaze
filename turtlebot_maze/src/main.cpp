#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <turtlebot_srv/ObstacleAvoidance.h>
#include <turtlebot_srv/PotentialField.h>
#include <turtlebot_srv/MazeCompletion.h>
#include <turtlebot_srv/Turn.h>
#include <turtlebot_srv/Align.h>
#include <turtlebot_srv/Planner.h>
#include <iostream>
#include <math.h>

class MainControl{

private:
	//subscribers and publishers
	ros::NodeHandle nh_;
	ros::Subscriber odom_sub;
	ros::Subscriber laser_sub;
	ros::Publisher vel_pub;


public: 
	//services
	ros::ServiceClient turn_srv;

	//messages
	geometry_msgs::Twist vel_msg;
	geometry_msgs::Twist vel_vec;
	turtlebot_srv::Turn turn_msg;

	//robot odometry
	double x_pos;
	double y_pos;
	double yaw_angle;

	//decision making
	std::vector<double> waypoint;
	std::vector<double> next_move;
	int next_choice;
	bool left_option;
	bool center_option;
	bool right_option;

	//obstacles
	double left_ob;
	double center_ob;
	double right_ob;

	//goal status
	std::vector<double> goal_vec;

	//controller
	double yaw_ref;
	

	//constructor
	MainControl(){
		ROS_INFO("Main Control Initiated");
		//publishers and subscribers
		vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
		odom_sub = nh_.subscribe("odom", 1000, &MainControl::OdomCallback, this);
        laser_sub = nh_.subscribe("scan", 1000, &MainControl::LaserCallback, this);

        //services
        turn_srv = nh_.serviceClient<turtlebot_srv::Turn>("/turn_srv");

        //variable initialize
        waypoint.push_back(0.0);
        waypoint.push_back(0.0);
        left_ob = 0.5;
        right_ob = 0.5;
        center_ob = 3.5;
	}

	//functions
	void UpdateMovement()
	{
		double x_pos_copy = x_pos;
		double y_pos_copy = y_pos;
		if (sqrt(pow(x_pos_copy-waypoint[0],2) + pow(y_pos_copy-waypoint[1],2) < 0.1)){
			FindOptions();
			DecideMove();
			MakeTurn();
			UpdateWaypoint();
			CalcYawRef();
	
			ROS_INFO("next move: %i", next_choice);
			ROS_INFO("next waypoint: %f, %f", waypoint[0], waypoint[1]);
			ROS_INFO("yaw ref: %f", yaw_ref);
		}
	
		Controller();
	}

    void FindOptions()
	{

	    if (left_ob > 2.0){
	        left_option = true;
	        ROS_INFO("left option valid");
	    }
	    else{
	        left_option = false;
	    }

	    if (center_ob > 2.0){
	    	ROS_INFO("center option valid");
	        center_option = true;
	    }
	    else{
	        center_option = false;
	    }

	    if (right_ob > 2.0){
	    	ROS_INFO("right option valid");
	        right_option = true;
	    }
	    else{
	        right_option = false;
	    }
	}

    void DecideMove()
	{
	    double left_dot = goal_vec[0]*cos(yaw_angle+M_PI/2) + goal_vec[1]*sin(yaw_angle+M_PI/2);
	    double center_dot = goal_vec[0]*cos(yaw_angle) + goal_vec[1]*sin(yaw_angle);
	    double right_dot = goal_vec[0]*cos(yaw_angle+3*M_PI/2) + goal_vec[1]*sin(yaw_angle+3*M_PI/2);

	    next_move.clear();

	    if (left_option == true){
	        next_move.push_back(left_dot);
	    }
	    else{
	        next_move.push_back(-1000);
	    }

	    if (center_option == true){
	        next_move.push_back(center_dot);
	    }
	    else{
	        next_move.push_back(-1000);
	    }

	    if (right_option == true){
	        next_move.push_back(right_dot);
	    }
	    else{
	        next_move.push_back(-1000);
	    }

	    int maxElementIndex = std::max_element(next_move.begin(),next_move.end()) - next_move.begin();
	    double maxElement = *std::max_element(next_move.begin(), next_move.end());
	    next_choice = maxElementIndex;  
	}

    void MakeTurn()
	{
	    turn_msg.request.direction = next_choice;

	    if (turn_srv.call(turn_msg))
	    {
	        ROS_INFO("turning...");
	    }
	    else{
	        ROS_ERROR("could not send through request");
	    }
	}

    void UpdateWaypoint()
	{
	    waypoint.clear();
	    ros::spinOnce();

	    if (fabs(yaw_angle - 0.0) < 0.2){
	        ROS_INFO("0 degrees");
	        waypoint.push_back(x_pos + center_ob*cos(yaw_angle) - 0.5);
	        waypoint.push_back(y_pos);
	    }
	    else if (fabs(yaw_angle - 3*M_PI/2) < 0.4){
	        ROS_INFO("270 degrees");

	        waypoint.push_back(x_pos);
	        waypoint.push_back(y_pos + center_ob*sin(yaw_angle) + 0.5);
	    }
	    else if (fabs(yaw_angle - M_PI) < 0.2){
	        ROS_INFO("90 degrees");
	        waypoint.push_back(x_pos + center_ob*cos(yaw_angle) + 0.5);
	        waypoint.push_back(y_pos);
	    }
	    else if (fabs(yaw_angle - M_PI/2) < 0.2){
	        ROS_INFO("180 degrees");
	        waypoint.push_back(x_pos);
	        waypoint.push_back(y_pos + center_ob*sin(yaw_angle) - 0.5);
	    }
	}

	void CalcYawRef()
	{
		yaw_ref = atan2(waypoint[1]-y_pos,waypoint[0]-x_pos);
	}

	void Controller()
	{
		double Kp = 0.5;
		double yaw_angle_copy;

		if (yaw_angle > M_PI){
			yaw_angle_copy = yaw_angle - 2*M_PI;
		}
		else{
			yaw_angle_copy = yaw_angle;
		}

		ROS_INFO("yaw_ref, yaw_angle: %f, %f", yaw_ref, yaw_angle_copy);

		double yaw_act = -Kp*(yaw_angle_copy - yaw_ref);

		vel_msg.angular.z = yaw_act;
		vel_msg.linear.x = 0.5;
		vel_pub.publish(vel_msg);

	}

    void VectorUpdate()
	{   
	    goal_vec.clear();
	    double x_goal = -1.76;
	    double y_goal = -6.62;
	    goal_vec.push_back(x_goal - x_pos);
	    goal_vec.push_back(y_goal - y_pos);
	}

	//function callbacks
	void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        double q0 = msg->pose.pose.orientation.x;
        double q1 = msg->pose.pose.orientation.y;
        double q2 = msg->pose.pose.orientation.z;
        double q3 = msg->pose.pose.orientation.w;

        yaw_angle = atan2(2*(q3*q2+q0*q1),1-2*(pow(q1,2)+pow(q2,2)));
        if (yaw_angle < 0){
            yaw_angle = yaw_angle + 2*M_PI;
        }

        x_pos = msg->pose.pose.position.x;
        y_pos = msg->pose.pose.position.y;
    }

    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
	    left_ob = msg->ranges[90];
	    center_ob = msg->ranges[0];
	    right_ob = msg->ranges[270];

	    if (isinf(left_ob)){
	    	left_ob = 3.5;
	    }

	    if (isinf(center_ob)){
	    	center_ob = 3.5;
	    }

	    if (isinf(right_ob)){
	    	right_ob = 3.5;
	    }

	}


};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    MainControl main_control;

    while(ros::ok()){
        main_control.VectorUpdate();
        main_control.UpdateMovement();
        ros::spinOnce();
    }
    

    ros::spin();
    return 0;
}