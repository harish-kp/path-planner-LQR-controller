#include  <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include<Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;
#include<vector>
#include<math.h>
// #include <sensor_msgs>
#include "matplotlib/matplotlibcpp.h"
// #include "madplotlibcpp.h"
#include "iostream"



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
    // odom_sub = nh->subscribe<nav_msgs::Odometry>("/odom", 10, odom_callback);
    // odom_msg = nav_msgs::Odometry();
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
    // void odom_callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
    // {
    //     double odom_msg = pose_msg->pose.pose.position.z;
    //     odom_updated = true; 
    // }
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
    // typedef {VectorXf|ArrayXf} VectorXi;
    
    double T = 150; double num_steps = 15; int n = 3; int m = 2;int p = 3;
    Eigen::VectorXd t = (Eigen::VectorXd::LinSpaced(num_steps,0.0,135.0));
    Eigen::VectorXd t_ten = t/10;
    Eigen::VectorXd t_twenty = t/20;
    Eigen::VectorXd x1 = t_ten.Eigen::VectorXd::array().sin();
    // std::cout << x1 << std::endl;
    Eigen::VectorXd x2 = t_twenty.Eigen::VectorXd::array().sin();
    // matplotlibcpp::plot (x1,x2);
    // matplotlibcpp::show();
    Eigen::MatrixXd parametric_func = Eigen::MatrixXd::Zero(2,num_steps);
    parametric_func.topRows(1) = x1.transpose();
    parametric_func.bottomRows(1) = x2.transpose();
    std::cout << parametric_func << std::endl;
    double dt = T/num_steps;
    Eigen::MatrixXd s = Eigen::MatrixXd::Zero(2,num_steps);
    Eigen::MatrixXd stemp = Eigen::MatrixXd::array();

    ROS_INFO_STREAM ("Create LQR Controller node");
    ROS_INFO_STREAM ("..........................");
    ros::init(argc, argv, "lqrController");
    ros::NodeHandle nh;
    
    lqrController robot = lqrController(&nh);
    while (ros::ok()){
    for (int i = 0; i < num_steps; i++){
        robot.lqrLoop();        
    } 
    robot.robot_stop();
    ros::spinOnce();
    }
    return 0;
}