#!/usr/bin/env python3
import numpy as np
import rospy
import roslib
import transforms3d
import math
# from transforms3d import euler.euler2quat as euler2quat
from control import lqr
import copy
import time
import matplotlib.pyplot as plt
import time
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
# from transforms3d import euler2quat

#Initialize Controller Variables
print("Initializing Controller Variables")
print("................................")

#Change T and num_steps to modify runtime and dt (dt = T/num_steps)
T = 150
num_steps = 15000

n = 3
m = 2
p = 3

pMinusS = np.array([2])
A = np.identity(3)
B = np.array([[1, 0],
              [1, 0],
              [0, 1]])
C = np.identity(3)
Sigma_w = np.array([[1e-6, 0, 0],
                    [0, 1e-6, 0],
                    [0, 0, 1e-6]])
Sigma_v = np.array([[1e-6, 0, 0],
                    [0, 1e-6, 0],
                    [0, 0, 1e-6]])
Q = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])
R = np.array([[1, 0],
              [0, 1]])

rd_tar = 1
rd_obs = 1
target = np.array([2, 0.001, 0])
obs = np.array([-1, 1])

#Reference trajectory of a "figure eight" shape
t = np.linspace(0.0, 135.0, num = num_steps)
x1 = np.sin(t/10)
x2 = np.sin(t/20)

parametric_func = np.zeros((2,num_steps))
parametric_func[0] = x1
parametric_func[1] = x2

dt = float(T)/float(num_steps)
s = np.zeros((2, num_steps))
stemp = np.array([[0],[0]])
b = np.zeros((2,2,num_steps))
s[:,num_steps-1]=[0,0]
A_l = np.identity(2)
B_l = dt*np.identity(2)
Q_l = np.identity(2)
degree = 3
B_lh = B_l.conj().transpose()
g_D = rd_tar^2
g_U = rd_obs^2

ref_traj = parametric_func
diffrc = ref_traj[:,0]

ref_length = len(ref_traj[1])
ref_traj = np.concatenate((ref_traj, np.ones((1,ref_length))))

for i in range(0,ref_length-1):
    ref_traj[2,i] = np.arctan((ref_traj[1,i+1]-ref_traj[1,i])/(ref_traj[0,i+1]-ref_traj[0,i]))
ref_traj[2,ref_length-1] = ref_traj[2,ref_length-2]

ref_traj_dot = np.zeros((3,ref_length))
for i in range (1, ref_length):
    ref_traj_dot[0,i] = (ref_traj[0,i]-ref_traj[0,i-1])/dt
    ref_traj_dot[1,i] = (ref_traj[1,i]-ref_traj[1,i-1])/dt
    ref_traj_dot[2,i] = (ref_traj[2,i]-ref_traj[2,i-1])/dt

ref_traj_db_dot = np.zeros((3,ref_length))
for i in range(0, ref_length-1):
    ref_traj_db_dot[0,i] = (ref_traj_dot[0,i+1]-ref_traj_dot[0,i])/dt
    ref_traj_db_dot[1,i] = (ref_traj_dot[1,i+1]-ref_traj_dot[1,i])/dt
    ref_traj_db_dot[2,i] = (ref_traj_dot[2,i+1]-ref_traj_dot[2,i])/dt

ref_length = len(ref_traj[2])
rd = np.zeros((2,ref_length-1))
for i in range (0, ref_length-1):
    rd[0,i] = ref_traj[0,i+1]-ref_traj[0,i]
    rd[1,i] = ref_traj[1,i+1]-ref_traj[1,i]

rdd = np.zeros((2,ref_length-2))
for i in range(0,ref_length-2):
    rdd[0,i] = rd[0,i+1]-rd[0,i]
    rdd[1,i] = rd[1,i+1]-rd[1,i]
# print(ref_traj)
# redefine start point and target
# start_point = ref_traj(:,1)
start_point = ref_traj[:,0]
# print (start_point)
target = ref_traj[:,ref_length-1]

if dt <= 0:
    dt = 1e-4

if n <= 0:
    n = 2

if m <= 0:
    m = 2

if p <= 0:
    p = 2

[rowA, colA] = A.shape
if rowA != n or colA != n:
    print('A should be an n*n matrix.')

[rowB, colB] = B.shape
if rowB != n or colB != m:
    print('B should be an n*m matrix.')

[rowC, colC] = C.shape
if rowC != p or colC != n:
    print('C should be an p*n matrix.')

[rowQ, colQ] = Q.shape
if rowQ != n or colQ != n:
    print('Q should be an n*n matrix.')

[rowR, colR] = R.shape
if rowR != m or colR != m:
    print('R should be an m*m matrix.')

C_alpha = C[pMinusS-1,:]
Sigma_v_alpha = Sigma_v[pMinusS-1, pMinusS-1]
R_inv = np.linalg.inv(R)
Sigma_v_inv = np.linalg.inv(Sigma_v)

x_hat = np.zeros((n, num_steps))
x_alpha_hat = np.zeros((n,num_steps))
x_real = np.zeros((n,num_steps))
x0 = start_point
x_hat[:,0] = x0
x_alpha_hat[:,0] = x0
x_real[:,0] = x0

G = Sigma_w

# P = np.zeros((n,n,num_steps))

# Sigma_x = np.zeros((n, n, num_steps))

u = np.zeros((m, num_steps))
y = np.zeros((p, num_steps))

# x_p = np.zeros((n,1))
# Sigma_x_p = np.zeros((n,n))

error = np.zeros((1,num_steps))
# cost = np.zeros((1,num_steps))
finish = False

#Calculate s matrix
for j in range(num_steps-2, -1, -1):
    k,s,e = lqr(A,B,Q,R)
    print (k)
    print (s)
    k1 = -(np.linalg.inv(B_l.conj().transpose()*b[:,:,j+1]*B_l+R)*B_l.conj().transpose()*b[:,:,j+1])*A_l
    print (k1)
    b[:,:,j] = A_l.conj().transpose()*(b[:,:,j+1]-b[:,:,j+1]*B_l*np.linalg.inv(B_l.conj().transpose()*b[:,:,j+1]*B_l+R)*B_l.conj().transpose()*b[:,:,j+1])*A_l+Q_l
    ref_traj_a = np.array([[ref_traj[0,j+1]],[ref_traj[1,j+1]]])
    # stemp = np.matmul((A_l.conj().transpose() + k*B_l.conj().transpose()),stemp) - np.matmul(Q_l,ref_traj_a)
    stemp[0,j] = s[0]
    stemp[1,j] = s[1]

first_step_angle = np.arctan((ref_traj[1,1] - ref_traj[1,0])/(ref_traj[0,1] - ref_traj[0,0]))
init_angle = 0
theta = first_step_angle-init_angle
state_init = [0,0,1e-4]
B_ind = 0

B = np.array([[np.cos(0.0079),0],
              [np.sin(0.0079),0],
              [0,1]])
y[:,0] = state_init

# P = np.zeros(b.shape)
# for i in range (0,num_steps):
#     P[:,:,i] = np.subtract(b[:,:,i],Q_l)

Phi = np.zeros((n,n,num_steps))
Theta = np.zeros((n,p,num_steps))
Theta[:,:,0] = Phi[:,:,0]*C.conj().transpose()*Sigma_v_inv

#Calculate initial control input u
uu1 = np.linalg.inv(np.matmul(np.matmul(B_lh,b[:,:,0]),B_l)+R)
uu2 = np.matmul(uu1,B_lh)
uu3 = (np.matmul(np.matmul(b[:,:,0],A_l),x_hat[0:2,0])+s[:,0])
uu3 = np.reshape(uu3,(2,1))
uu4 = np.matmul(uu2,uu3)/dt
uu4 = np.reshape(uu4,(2,1))
uu5 = np.reshape(ref_traj_db_dot[0:2,0]*dt,(2,1)) - uu4
u[:,0] = np.reshape(uu5,(1,2))

start_time = 0
elapsed_time = 0

#Plot graphs
def plotting():
    global ref_traj, y, dt, T, num_steps, elapsed_time
    plt.ioff()

    #Reference trajectory vs Actual trajectory
    fig1 = plt.figure()
    fig1.suptitle("Reference trajectory vs Actual trajectory\n " + "dt = " + str(dt) + " T = " + str(T) + " num_steps = " + str(num_steps) + " Elapsed time: " + str(elapsed_time))
    plt.plot(ref_traj[0,:], ref_traj[1,:], label = 'Reference trajectory')
    plt.plot(y[0,:], y[1,:], label = 'Actual trajectory')
    plt.show()


#LQR controller class, initialize node, publishers, and subscribers
class lqr_controller:
    def __init__(self):
        print("Creating LQR Controller Node")
        print("............................")
        rospy.init_node('LQR_Controller')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2) # Publish a twist message in order to control the robot's movement
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback) # subscribe to the robot's IMU in order to recieve position data
        self.odom_msg = Odometry()
        self.pose_msg = Pose()
        self.vel_msg = Twist()
        self.odom_updated = False

    def odom_callback(self, msg): # Callback functions within the subscriber run continuously in order to update the relevant information
        self.odom_msg = msg
        self.odom_updated = True

    def lqr_loop(self, msg, i):
        global A, B, Xi, omega,n,x_hat,dt
        #Calculate initial Xi and omega
        if i == 0:
            stime1 = time.time()
            Xi = u[0,0]*np.cos(x_hat[2,0])*dt+u[1,0]*np.sin(x_hat[2,0])*dt
            omega = dt*(u[1,0]*np.cos(x_hat[2,0])-u[0,0]*np.sin(x_hat[2,0]))/Xi
            self.vel_msg.linear.x = Xi
            self.vel_msg.angular.z = omega
            self.vel_pub.publish(self.vel_msg)
            elapsed = time.time() - stime1

            #Loop until elapsed >= dt
            while elapsed < dt:
                elapsed = time.time() - stime1
        else:
            stime1 = time.time() #start stopwatch for one single step
            x_temp = np.matmul(x_hat[:,i-1],A).reshape(-1, 1) + np.matmul(B,np.array([[1.5*Xi*dt],[omega*dt]]))
            A_ext = np.array([[1, 0, -dt*Xi*np.sin(x_hat[2,i-1])],
                     [0, 1, dt*Xi*np.cos(x_hat[2,i-1])],
                     [0, 0, 1]])
            Phi_temp = np.matmul(np.matmul(A_ext,Phi[:,:,i-1]),A_ext.conj().transpose())+Sigma_w

            #Get positions from Turtlebot
            tbot_x = msg.pose.pose.position.x
            tbot_y = msg.pose.pose.position.y
            quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            angles = transforms3d.euler.euler2quat(quat)
            y[:,i] = [tbot_x, tbot_y, angles[2]]

            z = y[:,i].reshape(-1,1)-np.matmul(C,x_temp)
            s_temp = np.matmul(np.matmul(C,Phi_temp),C.conj().transpose())+Sigma_v
            Theta[:,:,i] = np.matmul(np.matmul(Phi_temp,C.conj().transpose()),np.linalg.inv(s_temp))
            x_hat[:,i] = np.reshape(x_temp + np.matmul(Theta[:,:,i],z),3)
            Phi[:,:,i] = np.matmul((np.identity(n) - np.matmul(Theta[:,:,i],C)),Phi_temp)
            B = np.array([[np.cos(x_hat[2,i]),0],[np.sin(x_hat[2,i]),0],[0,1]])

            #Calculate control input u
            uu1 = np.linalg.inv(np.matmul(np.matmul(B_lh,b[:,:,i]),B_l)+R)
            uu2 = np.matmul(uu1,B_lh)
            uu3 = (np.matmul(np.matmul(b[:,:,i],A_l),x_hat[0:2,i])+s[:,i])
            uu3 = np.reshape(uu3,(2,1))
            uu4 = np.matmul(uu2,uu3)/dt
            uu4 = np.reshape(uu4,(2,1))
            uu5 = np.reshape(ref_traj_db_dot[0:2,i]*dt,(2,1)) - uu4
            u[:,i] = np.reshape(uu5,(1,2))

            #Calculate Xi and omega
            Xi = u[0,i]*np.cos(x_hat[2,i])*dt+u[1,i]*np.sin(x_hat[2,i])*dt
            if Xi != 0:
                omega = dt*(u[1,i]*np.cos(x_hat[2,i])-u[0,i]*np.sin(x_hat[2,i]))/Xi
            else:
                omega = 0

            #Publish message to velocity topic
            self.vel_msg.linear.x = Xi
            self.vel_msg.angular.z = omega
            self.vel_pub.publish(self.vel_msg)
            elapsed = time.time() - stime1
            while elapsed < dt:
                elapsed = time.time() - stime1

#Main function
if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        start_time = time.time()
        Robot = lqr_controller()
        for i in range(0, num_steps):
            Robot.lqr_loop(Robot.odom_msg,i)
        Robot.vel_msg.linear.x = 0
        Robot.vel_msg.angular.z = 0
        Robot.vel_pub.publish(Robot.vel_msg)
        elapsed_time = time.time() - start_time
        plotting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
