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
        ros::ServiceClient turn_srv;
        ros::ServiceClient align_srv;
        ros::ServiceClient planner_srv;
        turtlebot_srv::PotentialField ob_srv;
        turtlebot_srv::Turn turn_msg;
        turtlebot_srv::Align align_msg;
        turtlebot_srv::Planner planner_msg;
        geometry_msgs::Twist vel_vec;
        std::vector<double> waypoint;
        std::vector<double> laser_data;

        //obstacles
        double left_ob;
        double center_ob;
        double right_ob;
        double theta_ref;
        double vel_vec_x;
        double vel_vec_y;

        //goal status
        std::vector<double> goal_vec;

        //robot odometry
        double x_pos;
        double y_pos;
        double yaw_angle;

        //decision making
        bool left_option;
        bool center_option;
        bool right_option;
        std::vector<double> next_move;
        int next_choice;

        //planner
        std::vector<float> optimal_path;
        std::vector<float> begin_end_coords;

        
        MainControl(){
            ROS_INFO("Main control initiated");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
            obstacle_avoidance_srv = nh_.serviceClient<turtlebot_srv::PotentialField>("/obstacle_avoidance_srv");
            maze_completion_srv = nh_.serviceClient<turtlebot_srv::MazeCompletion>("maze_completion_srv");
            align_srv = nh_.serviceClient<turtlebot_srv::Align>("/align_srv");
            planner_srv = nh_.serviceClient<turtlebot_srv::Planner>("/planner_srv");
            turn_srv = nh_.serviceClient<turtlebot_srv::Turn>("/turn_srv");
            odom_sub = nh_.subscribe("odom", 1000, &MainControl::OdomCallback, this);
            laser_sub = nh_.subscribe("kobuki/laser/scan", 1000, &MainControl::LaserCallback, this);
            waypoint.push_back(0.0);
            waypoint.push_back(0.0);
            left_ob = 0.6;
            center_ob = 4.7;
            right_ob = 0.6;
        }

        void CheckObstacle()
        {   
            ob_srv.request.goal.position.x = waypoint[0];
            ob_srv.request.goal.position.y = waypoint[1];
            if (obstacle_avoidance_srv.call(ob_srv))
            {
                vel_vec = ob_srv.response.vel_vec;
                //ROS_INFO("velocity vector: %f, %f", vel_vec.linear.x, vel_vec.linear.y);
            }
            else{
                ROS_ERROR("could not send through request");
            }
        }

        void UpdateMovement()
        {   
            //ROS_INFO("position: %f, %f", x_pos, y_pos);
            if (sqrt(pow(x_pos-waypoint[0],2) + pow(y_pos-waypoint[1],2) < 0.1)){
                ROS_INFO("update initiated");
                FindOptions();
                DecideMove();
                MakeTurn();
                UpdateWaypoint();
                begin_end_coords.push_back(x_pos);
                begin_end_coords.push_back(y_pos);
                begin_end_coords.push_back(waypoint[0]);
                begin_end_coords.push_back(waypoint[1]);
                FindPath(begin_end_coords);

                CheckObstacle();
                ROS_INFO("waypoint: %f, %f", waypoint[0], waypoint[1]);
            }
            //ROS_INFO("position: %f, %f", x_pos, y_pos);
            if (center_ob < 0.2 || right_ob < 0.2 || left_ob < 0.2){
                CheckObstacle();
                theta_ref = atan2(vel_vec.linear.y, vel_vec.linear.x);
                AlignYaw();
            }
    
            double x_vec = vel_vec.linear.x*cos(yaw_angle) + vel_vec.linear.y*sin(yaw_angle);
            double y_vec = -vel_vec.linear.x*sin(yaw_angle) + vel_vec.linear.y*cos(yaw_angle);
            
            //ROS_INFO("robot vec: %f, %f", x_vec, y_vec);
            //ROS_INFO("theta_ref: %f", theta_ref);
            //ROS_INFO("yaw_angle: %f", yaw_angle);
            vel_msg.linear.x = x_vec;
            //vel_msg.linear.y = y_vec;
            
            vel_pub.publish(vel_msg);
        }

        void FindOptions()
        {
            //ROS_INFO("obstacles: %f, %f, %f", left_ob, center_ob, right_ob);

            if (left_ob > 1.0){
                left_option = true;
            }
            else{
                left_option = false;
            }

            if (center_ob > 1.0){
                center_option = true;
            }
            else{
                center_option = false;
            }

            if (right_ob > 1.0){
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
            //ROS_INFO("next choice: %i", next_choice);    

        }

        void UpdateWaypoint()
        {
            waypoint.clear();
            ros::spinOnce();
            //ROS_INFO("current yaw angle:: %f", yaw_angle);

            if (center_ob > 30){
                center_ob = 30;
            }

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
            //ROS_INFO("next waypoint: %f, %f", waypoint[0], waypoint[1]);
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

            x_pos = msg->pose.pose.position.x;
            y_pos = msg->pose.pose.position.y;
        }

        void MakeTurn()
        {
            turn_msg.request.direction = next_choice;

            if (turn_srv.call(turn_msg))
            {
                //ROS_INFO("turning...");
            }
            else{
                ROS_ERROR("could not send through request");
            }
        }

        void AlignYaw()
        {
            ROS_INFO("theta_ref: %f", theta_ref);
            align_msg.request.theta_ref = theta_ref;

            if (fabs(yaw_angle - theta_ref) > 0.1){
                if (align_srv.call(align_msg))
                {
                    ROS_INFO("align initiated");         
                }
                else{
                    ROS_ERROR("could not align");
                }
            }
        }

        void FindPath(std::vector<float> begin_end_coords){
            planner_msg.request.coords_msg = begin_end_coords;
            if (planner_srv.call(planner_msg)){
                ROS_INFO("getting path");
                optimal_path = planner_msg.response.optimal_path;
            }
            else{
                ROS_ERROR("can't get path");
            }
        }

        void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
            left_ob = msg->ranges[719];
            center_ob = msg->ranges[360];
            right_ob = msg->ranges[0];
            //ROS_INFO("obstacles: %f, %f, %f", left_ob, center_ob, right_ob);
        }

        void VectorUpdate()
        {   
            goal_vec.clear();
            double x_goal = 0.38;
            double y_goal = -8.54;
            goal_vec.push_back(x_goal - x_pos);
            goal_vec.push_back(y_goal - y_pos);
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_control_node");
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