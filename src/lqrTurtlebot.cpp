#include "ros.h"
#include "stdio.h"
class lqrController{
    public:

    std::cout << "Create LQR Controller node";
    std::cout << "..........................";
    ros::init(argc, argv, "lqrController");
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    vel_pub = nh.advertise<Twist>("/cmd_vel", 10);
    ros::Subscriber odom_sub;
    odom_sub = nh.subscribe<Odometry>("/odom", 10, odom_callback);
    odom_msg = Odometry();
    pose_msg = Pose();
    vel_msg = Twist();
    bool odom_updated = false;


    void odom_callback(const Odometry &msg)
    {
        odom_msg = msg;
        odom_updated = true; 
    }  
    
    void odom_msg{

    }
    int lqrLoop{

    }


}
int main {
    int T = 150; num_steps = 15000; n = 3; m = 2;p = 3;   
    robot = lqrController();
    for (int i = 0; i < num_steps; i++){
        robot.lqrLoop()
    } 
}