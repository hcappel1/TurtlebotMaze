#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_srv/Align.h>
#include <nav_msgs/Odometry.h>

class AlignRobot
{
    public:
        ros::NodeHandle nh_;
        ros::Publisher vel_pub;
        geometry_msgs::Twist vel_msg;

        //robot odometry
        double yaw_angle;
        double x_pos;
        double y_pos;

        AlignRobot()
        {
            ROS_INFO("aligning...");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        }

        bool Align(turtlebot_srv::Align::Request &req,
                turtlebot_srv::Align::Response &res)
        {
            while (fabs(yaw_angle - req.theta_ref) > 0.2){
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = -1.0*(Sign(yaw_angle - req.theta_ref));
                    vel_pub.publish(vel_msg);
                    ros::spinOnce();
                    ROS_INFO("yaw angle: %f", yaw_angle);
                }
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);

            res.success = true;
            return true;

            
        }

        int Sign(double x){
            if (x < 0){
                return -1;
            }
            else{
                return 1;
            }
        }

        void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            double q0 = msg->pose.pose.orientation.x;
            double q1 = msg->pose.pose.orientation.y;
            double q2 = msg->pose.pose.orientation.z;
            double q3 = msg->pose.pose.orientation.w;

            yaw_angle = atan2(2*(q3*q2+q0*q1),1-2*(pow(q1,2)+pow(q2,2)));
            x_pos = msg->pose.pose.position.x;
            y_pos = msg->pose.pose.position.y;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_srv_node");
    ros::NodeHandle nh;

    AlignRobot align_robot;

    ros::ServiceServer align_srv = nh.advertiseService("/align_srv", &AlignRobot::Align, &align_robot);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, &AlignRobot::OdomCallback, &align_robot);

    ros::spin();

    return 0;
}