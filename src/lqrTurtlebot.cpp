#include  <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <sensor_msgs>

#include "iostream"
#include "vector"

class lqrController{
    private:
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist msg;

    public:

    
    lqrController(ros::NodeHandle *nh){
    
    vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }
    int lqrLoop(){
       msg.linear.x = 0.5;
       vel_pub.publish(msg);}
       
    };
    
//     ros::Subscriber odom_sub;
//     odom_sub = nh.subscribe<Odometry>("/odom", 10, odom_callback);
//     odom_msg = Odometry();
//     pose_msg = Pose();
//     vel_msg = Twist();
//     bool odom_updated = false;


//     void odom_callback(const Odometry &msg)
//     {
//         odom_msg = msg;
//         odom_updated = true; 
//     }  
    
//     void odom_msg{

//     }
    
// }
// int ref_traj_gen{

// };
int main (int argc, char** argv){
    int T = 150; int num_steps = 15; int n = 3; int m = 2;int p = 3;   
    float A[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; float C[3][3]= {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float B[3][2] = {{1,0}, {1,0}, {0,1}}; float R[2][2]={{1,0}, {0,1}}; 
    float Q[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    ROS_INFO_STREAM ("Create LQR Controller node");
    ROS_INFO_STREAM ("..........................");
    ros::init(argc, argv, "lqrController");
    ros::NodeHandle nh;
    
    lqrController robot = lqrController(&nh);
    while (ros::ok()){
    for (int i = 0; i < num_steps; i++){
        robot.lqrLoop();        
    } 
    ros::spinOnce();
    }
    return 0;
}