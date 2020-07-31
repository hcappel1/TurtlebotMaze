#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_srv/Turn.h>
#include <nav_msgs/Odometry.h>

class Turn
{
    public:
        ros::NodeHandle nh_;
        ros::Publisher vel_pub;
        geometry_msgs::Twist vel_msg;

        //robot odometry
        double yaw_angle;
        double x_pos;
        double y_pos;

        //turn parameters
        double turn_angle;



        Turn(){
            ROS_INFO("turn activated");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        }

        bool MakeTurn(turtlebot_srv::Turn::Request &req,
                turtlebot_srv::Turn::Response &res)
        {
            if (req.direction == 0){
                turn_angle = M_PI/2;
                //if (turn_angle > 2*M_PI){
                  //  turn_angle -= 2*M_PI;
                //}
                while (fabs(yaw_angle - turn_angle) > 0.1){
                    vel_msg.angular.z = 0.55;
                    vel_pub.publish(vel_msg);
                    ros::spinOnce();
                    ROS_INFO("yaw angle: %f", yaw_angle);
                    //ROS_INFO("turn angle: %f", turn_angle);
                }
                vel_msg.angular.z = 0.0;
                vel_pub.publish(vel_msg);
                
            }
            else if (req.direction == 2){
                turn_angle = 3*M_PI/2;
                // if (turn_angle > 2*M_PI){
                //     turn_angle -= 2*M_PI;
                // }
                while (fabs(yaw_angle - turn_angle) > 0.1){
                    vel_msg.angular.z = -0.5;
                    vel_pub.publish(vel_msg);
                    ros::spinOnce();
                    ROS_INFO("yaw angle: %f", yaw_angle);
                    //ROS_INFO("turn angle: %f", turn_angle);
                }
                vel_msg.angular.z = 0.0;
                vel_pub.publish(vel_msg);
                
            }
            res.success = true;
            return true;
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

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_srv_node");
    ros::NodeHandle nh;

    Turn turn;

    ros::ServiceServer turn_srv = nh.advertiseService("/turn_srv", &Turn::MakeTurn, &turn);
    ros:: Subscriber odom_sub = nh.subscribe("odom", 1000, &Turn::OdomCallback, &turn);

    ros::spin();

    return 0;
}