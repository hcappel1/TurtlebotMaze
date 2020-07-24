#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <turtlebot_srv/ObstacleAvoidance.h>
#include <turtlebot_srv/PotentialField.h>
#include <turtlebot_srv/MazeCompletion.h>
#include <iostream>
#include <map>

class DecisionMaking
{
    private:
        ros::NodeHandle nh_;

    public:
        std::vector<std::vector<double>> search_data;

    DecisionMaking(){
        ROS_INFO("Decision making constructed");
    }
};


class MainControl
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub;
        ros::Subscriber laser_sub;

    public:
        ros::Publisher vel_pub;
        geometry_msgs::Twist vel_msg;
        ros::ServiceClient obstacle_avoidance_srv;
        ros::ServiceClient maze_completion_srv;
        turtlebot_srv::PotentialField ob_srv;
        geometry_msgs::Twist vel_vec;
        double time_stamp;
        double current_time;
        bool can_turn;
        double null_time;
        double robot_vel;
        double yaw_error;
        double yaw_ref;

        //robot odometry info
        double x_pos;
        double y_pos;
        double x_vel;
        double y_vel;
        double yaw_angle;

        //robot scan info
        double left_ob;
        double center_ob;
        double right_ob;

        //decision making info
        std::map<std::string,double> next_move;

        std::vector<double> waypoint;
        DecisionMaking decison_making;
        


        MainControl(){
            ROS_INFO("Main control initiated");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
            obstacle_avoidance_srv = nh_.serviceClient<turtlebot_srv::PotentialField>("/obstacle_avoidance_srv");
            maze_completion_srv = nh_.serviceClient<turtlebot_srv::MazeCompletion>("maze_completion_srv");
            time_stamp = ros::Time::now().toSec();
            odom_sub = nh_.subscribe("odom", 1000, &MainControl::OdomCallback, this);
            laser_sub = nh_.subscribe("scan", 1000, &MainControl::ScanCallback, this);
            waypoint.push_back(0.0);
            waypoint.push_back(0.0);
        }

        void CheckObstacle()
        {   
            ROS_INFO("waypoint: %f,%f", waypoint[0], waypoint[1]);
            ob_srv.request.goal.position.x = 3.88; //waypoint[0];
            ob_srv.request.goal.position.y = -0.27; //waypoint[1];
            if (obstacle_avoidance_srv.call(ob_srv))
            {
                vel_vec = ob_srv.response.vel_vec;
                //ROS_INFO("velocity vector: %f, %f", vel_vec.linear.x, vel_vec.linear.y);
            }
            else{
                ROS_ERROR("could not send through request");
            }
        }

        void UpdateMovementOld()
        {
            if (ros::Time::now().toSec() - time_stamp > 4.0){
                CheckObstacle();
                time_stamp = ros::Time::now().toSec();
            }
            
            
            yaw_ref = atan2(vel_vec.linear.y,vel_vec.linear.x);
            if (yaw_ref < 0){
                yaw_ref = yaw_ref + 2*M_PI;
            }
            ROS_INFO("yaw_ref: %f", yaw_ref*180/M_PI);
            ROS_INFO("yaw_act: %f", yaw_angle*180/M_PI);

            yaw_error = yaw_ref-yaw_angle;
            ROS_INFO("yaw error: %f", yaw_error);
            double Kp = 1.0;
            vel_msg.angular.z = Kp*yaw_error;

            if (abs(yaw_error) < 0.2){
                vel_msg.linear.x = 0.3;
            }
            else{
                vel_msg.linear.x = 0.0;
            }
            
            vel_pub.publish(vel_msg);

        }

        void UpdateMovement()
        {   
            if (sqrt(pow(x_pos-waypoint[0],2) + pow(y_pos-waypoint[1],1)) < 0.1){
                FindWaypoint();
                ROS_INFO("finding new waypoint");
            }
            CheckObstacle();
            vel_msg.linear.x = vel_vec.linear.x;
            vel_msg.linear.y = vel_vec.linear.y;
            vel_pub.publish(vel_msg);
        }

        void FindWaypoint()
        {
            //SearchZone();
            waypoint.clear();
            waypoint.push_back(3.88);
            waypoint.push_back(-0.27);
        }

        void SearchZoneOld()
        {
            double start_angle = yaw_angle;
            double start_time = ros::Time::now().toSec();

            while (true){

                if (abs(yaw_angle - start_angle) < 0.3 && ros::Time::now().toSec() - start_time > 3.0){
                    vel_msg.angular.z = 0.0;
                    vel_pub.publish(vel_msg);
                    ROS_INFO("successfully acquired data");
                    break;
                }
                else{
                    vel_msg.angular.z = 0.5;
                    vel_pub.publish(vel_msg);
                    ros::spinOnce();
                }
            }

        }

        void SearchZone()
        {
            if (left_ob > 4.0){
                next_move["left"] = left_ob;
            }
            else{
                next_move["left"] = 0.0;
            }

            if (center_ob > 4.0){
                next_move["center"] = center_ob;
            }
            else{
                next_move["center"] = 0.0;
            }

            if (right_ob > 4.0){
                next_move["right"] = right_ob;
            }
            else{
                next_move["right"] = 0.0;
            }
        }

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

            robot_vel = msg->twist.twist.linear.x;
            x_pos = msg->pose.pose.position.x;
            y_pos = msg->pose.pose.position.y;
            x_vel = msg->twist.twist.linear.x;
            y_vel = msg->twist.twist.linear.y;


        }

        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
            left_ob = msg->ranges[719];
            center_ob = msg->ranges[360];
            right_ob = msg->ranges[0];

        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_control_node");
    ros::NodeHandle nh;

    MainControl main_control;

    while(ros::ok()){
        main_control.UpdateMovement();
        ros::spinOnce();
    }
    

    ros::spin();
    return 0;
}