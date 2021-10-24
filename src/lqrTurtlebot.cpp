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
// #include "tensorMatrix.h"
// #include "boost/multi_array.hpp"s
#include <unsupported/Eigen/CXX11/Tensor>

// #include "madplotlibcpp.h"
#include "iostream"
template<typename T>
using  MatrixType = Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>;

template<typename T, typename Device = Eigen::DefaultDevice>
auto asEval(const Eigen::TensorBase<T, Eigen::ReadOnlyAccessors> &expr, // An Eigen::TensorBase object (Tensor, TensorMap, TensorExpr... )
            const Device & device = Device()                            // Override to evaluate on another device, e.g. thread pool or gpu.
            ) {
    using Evaluator = Eigen::TensorEvaluator<const Eigen::TensorForcedEvalOp<const T>, Device>;
    Eigen::TensorForcedEvalOp<const T> eval = expr.eval();
    Evaluator                          tensor(eval, device);
    tensor.evalSubExprsIfNeeded(nullptr);
    return tensor;
}

template<typename T, typename sizeType, typename Device = Eigen::DefaultDevice>
auto MatrixCast(const Eigen::TensorBase<T, Eigen::ReadOnlyAccessors> &expr, const sizeType rows, const sizeType cols, const Device &device = Device()) {
    auto tensor  = asEval(expr, device);
    using Scalar = typename Eigen::internal::remove_const<typename decltype(tensor)::Scalar>::type;
    return static_cast<MatrixType<Scalar>>(Eigen::Map<const MatrixType<Scalar>>(tensor.data(), rows, cols));
}


class lqrController{
    private:
    int T = 150; int num_steps = 15; int n = 3; int m = 2;int p = 3; int i;   
    Eigen::MatrixXd A,C,Q = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2);
    double B[3][2] = {{1,0}, {1,0}, {0,1}};  
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
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2);
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
    // parametric_func = parametric_func.transpose();    
    // std::cout << parametric_func << std::endl;
    double dt = T/num_steps;
    Eigen::MatrixXd s = Eigen::MatrixXd::Zero(2,num_steps);
    Eigen::Tensor<double, 3> b(num_steps,2, 2);
    b.setZero();
    // std::array<double,3> offset = {0,0,0};         //Starting point
    std::array<double,3> extent = {1,2,2};       //Finish point
    std::array<double,2> shape2 = {2,2};
    // std::array<double,3> offset1 = {1,0,0};
    // std::cout <<  b.slice(offset1, extent).reshape(shape2) << std::endl;
    // std::cout << b_temp_mat << std::endl;
    // std::cout << "-----------------------------"<< std::endl;
    Eigen::MatrixXd A_l = Eigen::MatrixXd::Identity(2,2);
    // std::cout << A_l << std::endl;
    Eigen::MatrixXd B_l = dt*Eigen::MatrixXd::Identity(2,2);
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << B_l << std::endl;
    Eigen::MatrixXd Q_l = Eigen::MatrixXd::Identity(2,2);
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << Q_l << std::endl;
    double degree = 3;
    Eigen::MatrixXd B_lh = B_l.conjugate().transpose();
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << Q_l << std::endl;
    double ref_length = parametric_func.topRows(1).size();
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << ref_length << std::endl;
    Eigen::MatrixXd concat_matrix = Eigen::MatrixXd::Ones(1,ref_length);
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << concat_matrix << std::endl;
    Eigen::MatrixXd ref_traj(parametric_func.rows()+concat_matrix.rows(), parametric_func.cols()); 
    ref_traj << parametric_func, concat_matrix;
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << ref_traj << std::endl;
    // std::cout << ref_traj(1,1) << std::endl;
    // double temp = (ref_traj(1,2)- ref_traj(1,1))/(ref_traj(0,2)-ref_traj(0,1));
    // ref_traj (2,0) = std::atan(temp);
    for (int i=0; i <= ref_length-2; i++){
        double temp = (ref_traj(1,i+1)- ref_traj(1,i))/(ref_traj(0,i+1)-ref_traj(0,i));
        ref_traj (2,i) = std::atan(temp*(3.1514/180));
    }
    ref_traj(2,ref_length-1) = ref_traj(2,ref_length-2);
    Eigen::MatrixXd ref_traj_dot = Eigen::MatrixXd::Zero(3,ref_length);
    for (int i = 1; i <= ref_length-1; i++){
        ref_traj_dot(0,i) = (ref_traj(0,i) - ref_traj(0,i-1))/dt;
        ref_traj_dot(1,i) = (ref_traj(1,i) - ref_traj(1,i-1))/dt;
        ref_traj_dot(2,i) = (ref_traj(2,i) - ref_traj(2,i-1))/dt;
    }
    Eigen::MatrixXd ref_traj_db_dot = Eigen::MatrixXd::Zero(3,ref_length);
    for (int i = 0; i <= ref_length-2; i++){
        ref_traj_db_dot(0,i) = (ref_traj_dot(0,i+1) - ref_traj_dot(0,i))/dt;
        ref_traj_db_dot(1,i) = (ref_traj_dot(1,i+1) - ref_traj_dot(1,i))/dt;
        ref_traj_db_dot(2,i) = (ref_traj_dot(2,i+1) - ref_traj_dot(2,i))/dt;
    }
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << ref_traj.transpose().row(1) << std::endl;
    Eigen::MatrixXd start_point = ref_traj.transpose().row(0);
    Eigen::MatrixXd target = ref_traj.transpose().row(ref_length-1);
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << start_point << std::endl;
    // std::cout << target << std::endl;
    if (dt<=0){ 
        dt = 1e-4;
    }
    if (n<=0){ 
        n = 2;
    }
    if (m<=0){ 
        m = 2;
    }
    if (p<=0){ 
        p = 2;
    }
    Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(n,num_steps);
    Eigen::MatrixXd x0 = start_point;
    x_hat.col(0) = x0.transpose();
    // std::cout << "-----------------------------"<< std::endl;
    // std::cout << x_hat << std::endl;
    Eigen::MatrixXd u = Eigen::MatrixXd::Zero(m,num_steps);
    Eigen::MatrixXd y = Eigen::MatrixXd::Zero(p,num_steps);
    bool finish = false;

    // for (int i = num_steps-2; i > 0; i--){
    //     std::array<double,3> offset = {i+1,0,0};
    //     Eigen::Tensor<double, 2> b_temp = b.slice(offset, extent).reshape(shape2);
    //     Eigen::MatrixXd b_temp_mat = MatrixCast(b_temp,2,2);
    //     Eigen::MatrixXd k = -((B_l.transpose() * b_temp_mat * B_l + R).inverse()*B_l.transpose()*b_temp_mat)*A_l;
    //     // std::cout << k << std::endl;
    // }
    // std::cout << tensor << std::endl;
    // Eigen::MatrixXd stemp = Eigen::MatrixXd::array();

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