#include <ros/ros.h>
#include <turtlebot_srv/ObstacleAvoidance.h>
#include <turtlebot_srv/PotentialField.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>


class ObstacleAvoidance
{
    public:
        ros::NodeHandle nh_;
        double K_a;
        double K_r;
        std::vector<double> goal_pos;
        std::vector<double> robot_pos;
        double robot_angle;
        std::vector<double> range_data;
        double left_obstacle;
        double right_obstacle;
        double center_obstacle;


        ObstacleAvoidance()
        {
            ROS_INFO("Obstacle Avoidance Activated");
        }


        void ObstacleCheck(const sensor_msgs::LaserScan::ConstPtr &data)
        {
            for (int i = 0; i < 720; i++){
                range_data.push_back(data->ranges[i]);
            }
            right_obstacle = data->ranges[0];
            left_obstacle = data->ranges[719];
            center_obstacle = data->ranges[360];

        }

        void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            robot_pos.clear();
            robot_pos.push_back(msg->pose.pose.position.x);
            robot_pos.push_back(msg->pose.pose.position.y);

            double q0 = msg->pose.pose.orientation.x;
            double q1 = msg->pose.pose.orientation.y;
            double q2 = msg->pose.pose.orientation.z;
            double q3 = msg->pose.pose.orientation.w;

            robot_angle = atan2(2*(q3*q2+q0*q1),1-2*(pow(q1,2)+pow(q2,2)));
            if (robot_angle < 0){
                robot_angle = robot_angle + 2*M_PI;
            }
            //ROS_INFO("yaw: %f", robot_angle*(180/M_PI));
        }


        bool PotentialField(turtlebot_srv::PotentialField::Request &req,
                        turtlebot_srv::PotentialField::Response &res)
        {
            //variables
            double K_a = 0.75;
            double K_r = 0.01;
            double d_max = 0.1;
            double f_x = 0.0;
            double f_y = 0.0;
            double f_att_x = 0.0;
            double f_att_y = 0.0;
            double norm_att = 0.0;
            double f_rep_x = 0.0;
            double f_rep_y = 0.0;
            double ob_gain = 0.0;
            double ob_x = 0.0;
            double ob_y = 0.0;
            double ob_norm = 0.0;
            double d_angle = M_PI/720;



            //attractive field
            norm_att = sqrt(pow(req.goal.position.x - robot_pos[0],2)+pow(req.goal.position.y - robot_pos[1],2));
            f_att_x = K_a*(req.goal.position.x - robot_pos[0])/norm_att;
            f_att_y = K_a*(req.goal.position.y - robot_pos[1])/norm_att;

            f_x += f_att_x;
            f_y += f_att_y;

            //repulsive field
            if (sizeof(range_data) > 0){
                for (int i = 0; i < 720; i++){
                    if (range_data[i] < d_max){
                        ob_gain = -K_r*(1/range_data[i] - 1/d_max);
                        ob_x = range_data[i]*cos(robot_angle + 3*M_PI/2 + d_angle*i);
                        ob_y = range_data[i]*cos(robot_angle + 3*M_PI/2 + d_angle*i);
                        ob_norm = sqrt(pow(ob_x,2)+pow(ob_y,2));
                        f_rep_x = ob_gain*(ob_x/ob_norm);
                        f_rep_y = ob_gain*(ob_y/ob_norm);
                    }
                    else{
                        f_rep_x = 0.0;
                        f_rep_y = 0.0;
                    }
                    f_x += f_rep_x;
                    f_y += f_rep_y;
                }
            }
            else{
                ROS_ERROR("No obstacle check");
            }

            res.vel_vec.linear.x = f_x;
            res.vel_vec.linear.y = f_y;

            return true;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ros::NodeHandle nh;

    ObstacleAvoidance obstacle_avoidance;

    ros::ServiceServer obstacle_avoidance_srv = nh.advertiseService("/obstacle_avoidance_srv", &ObstacleAvoidance::PotentialField, &obstacle_avoidance);
    ros::Subscriber laser_sub = nh.subscribe("kobuki/laser/scan", 1000, &ObstacleAvoidance::ObstacleCheck, &obstacle_avoidance);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, &ObstacleAvoidance::OdomCallback, &obstacle_avoidance);
    ros::spin();

    return 0;
}
