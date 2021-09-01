#include  <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include<Eigen/Eigen>
#include<Eigen/Core>
// #include <sensor_msgs>
#include <matplotlib.h>
#include "iostream"
#include "vector"

class lqrController{
    private:
    int T = 150; int num_steps = 15; int n = 3; int m = 2;int p = 3; int i;   
    float A[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; float C[3][3]= {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float B[3][2] = {{1,0}, {1,0}, {0,1}}; float R[2][2]={{1,0}, {0,1}}; 
    float Q[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    bool odom_updated;

    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist msg;
    
    public:

    lqrController(ros::NodeHandle *nh){
    
    vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_sub = nh->subscribe<nav_msgs::Odometry>("/odom", 10, odom_callback);
    odom_msg = nav_msgs::Odometry();
    }
    int lqrLoop(){
        if (i < num_steps){
       msg.angular.z = 0.5;
       vel_pub.publish(msg);}}
    
    int robot_stop(){
        msg.linear.x = 0;
        msg.angular.z = 0;
        vel_pub.publish(msg);
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
    {
        double odom_msg = pose_msg->pose.pose.position.z;
        odom_updated = true; 
    }
};
//     ros::Subscriber odom_sub;
//     odom_sub = nh.subscribe<Odometry>("/odom", 10, odom_callback);
//     odom_msg = Odometry();
//     pose_msg = Pose();
//     vel_msg = Twist();
//     bool odom_updated = false;


      
    
//     void odom_msg{

//     }
    
// }
// int ref_traj_gen{

// };
int main (int argc, char** argv){
    int T = 150; int num_steps = 15; int n = 3; int m = 2;int p = 3;
    t = Eigen::
    ROS_INFO_STREAM ("Create LQR Controller node");
    ROS_INFO_STREAM ("..........................");
    ros::init(argc, argv, "lqrController");
    ros::NodeHandle nh;
    
    lqrController robot = lqrController(&nh);
    while (ros::ok()){
    for (int i = 0; i < num_steps; i++){
        robot.lqrLoop(robot.odom_callback,num_steps,i);        
    } 
    robot.robot_stop();
    ros::spinOnce();
    }
    return 0;
}