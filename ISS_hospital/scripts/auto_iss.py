#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, atan, pi, sqrt


# variables
v = 1
v_max = 1

n = 8 #nb of each traj's way points
n_traj = 15 #nb of trajs

rob_rad = 0.55
lid_rad = 4.5

dt = lid_rad/(n*v)
dphi = 0.45
t_traj =n*dt  #try dt=0.6

ang_ctrls = [] #contains all angular controls (for bound trajs)
traj_pts = [] #contains all trajs' way points
bound_list = [] #contains boundary states only

obs_coords = [] #contains obs coords /robot
obs_abs_coords = [] #contains obs coords /fixed ref
obs_ang = [] #contains obs' angles /lindar



# detect obstacles and obtain their coords /robot
def laserScanCallback(laser):
    global angY, angZ, linX, ranges
    global sub2

    ang_min = laser.angle_min
    ang_step = laser.angle_increment
    ranges = laser.ranges

    for i in range(len(ranges)):

        if 0.2<ranges[i]<4.45:
            ang = ang_min + i*ang_step
            x_obs = ranges[i]*cos(ang)
            y_obs = ranges[i]*sin(ang)

            obs_coords.append([x_obs, y_obs, 0])
            obs_ang.append(ang)

    sub1.unregister()


# generate controls to reach boundary states
def gen_controls(msg):

    #get current position
    x_curr = msg.pose.pose.position.x
    y_curr = msg.pose.pose.position.y

    orientation_quat_curr = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    _, _, theta_curr = euler_from_quaternion(orientation_quat_curr)

    start = [x_curr,y_curr,theta_curr]

    #transform obs coords to fixed ref
    T = [
        [cos(theta_curr), -sin(theta_curr), 0, start[0]],
        [sin(theta_curr), cos(theta_curr), 0, start[1]],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
    A = np.transpose(obs_coords)

    for i in range(len(obs_ang)):
        transformed_point = np.dot(T, np.append(A[:, i], [1]))
        obs_abs_coords.append([transformed_point[0], transformed_point[1]])
    
    #gen controls then associated bound trajs
    rospy.loginfo("currently looking for FORWARD safe trajectories.")
    for q in range(int(-(n_traj-1)/2), int((n_traj-1)/2)+1):
        ang_ctrls.append((dphi/dt)*(q/n_traj))
        gen_bound_traj(start, ang_ctrls[-1])

    if len(traj_pts)==0:
        rospy.loginfo("looking for RIGHT safe trajectories...")
        gen_right_traj(start)
        if len(traj_pts)==0:
            rospy.loginfo("looking for LEFT safe trajectories...")
            gen_left_traj(start)
    
    if len(traj_pts)==0:
        rospy.loginfo("NO safe trajectory avaible.")
        rospy.sleep(0.2)
        rospy.signal_shutdown('end task') #end node when no safe traj is available

    #plot all trajs
    plot()
    
    #apply a control
    apply_ctrls(msg)

    sub2.unregister()


# generate a safe boundary traj
def gen_bound_traj(start, w):
    x_curr, y_curr, theta_curr = start[0], start[1], start[2]
    safe = True

    #gen the traj
    for i in range(n):
        x_next = x_curr + v*dt*cos(theta_curr+(w*dt/2))
        y_next = y_curr + v*dt*sin(theta_curr+(w*dt/2))
        theta_next = theta_curr + w*dt

        #check safety: if obs detected, stop traj generation + if prohibited zone is not respected, stop traj gen
        # if i>3:
        if len(obs_ang)!=0:
            for j in range(len(obs_abs_coords)):
                d = sqrt((x_next-obs_abs_coords[j][0])**2 +(y_next-obs_abs_coords[j][1])**2)
                if d<rob_rad:
                    safe=False

        x_curr, y_curr, theta_curr = x_next, y_next, theta_next

    #save it if safe
    if not safe:
        ang_ctrls.pop() #remove the control for skipped traj
    if safe:
        x_curr, y_curr, theta_curr = start[0], start[1], start[2]
        
        for i in range(n):
            x_next = x_curr + v*dt*cos(theta_curr+(w*dt/2))
            y_next = y_curr + v*dt*sin(theta_curr+(w*dt/2))
            theta_next = theta_curr + w*dt
            x_curr, y_curr, theta_curr = x_next, y_next, theta_next
        
            traj_pts.append([x_next, y_next, theta_next])

    if len(traj_pts)!=0:
        bound_list.append(traj_pts[-1])

def gen_left_traj(start):
    for q in range(int(-(n_traj-1)/2), int((n_traj-1)/2)+1):
        ang_ctrls.append((dphi/dt)*(q/n_traj) +0.3/dt)  #(+ or -) 0.3/dt
        gen_bound_traj(start, ang_ctrls[-1])

def gen_right_traj(start):
    for q in range(int(-(n_traj-1)/2), int((n_traj-1)/2)+1):
        ang_ctrls.append((dphi/dt)*(q/n_traj) -0.3/dt)  #(+ or -) 0.3/dt
        gen_bound_traj(start, ang_ctrls[-1])


# plot all trajectories
def plot():

    x=[traj_pts[i][0] for i in range(len(traj_pts))]
    y=[traj_pts[i][1] for i in range(len(traj_pts))]

    plt.figure()
    plt.scatter(x,y)

    plt.xlabel('x coordinates')
    plt.ylabel('y coordinates')
    plt.title('all safe trajectories')
    
    # plt.show()
    plt.show(block=False)
    plt.pause(0.01)

    #rospy.signal_shutdown('task completed')


# apply the controls
def apply_ctrls(msg):
    
    msg=Twist()
    global leng

    leng = len(ang_ctrls)

    if leng!=0:
        if leng%2==0:
            twist_pub(v,ang_ctrls[int(leng/2)])
            rospy.sleep(t_traj/2)
        else:
            twist_pub(v,ang_ctrls[int((leng-1)/2)])
            rospy.sleep(t_traj/2)
        empty_lists()
        twist_pub(0,0)
        rospy.sleep(0.2)
    else :
        twist_pub(0,0) #just for testing, replace it later with braking
        rospy.signal_shutdown('no safe traj avaible') 

def twist_pub(v,w):
    
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    pub1.publish(msg)

def empty_lists():
    ang_ctrls.clear()
    traj_pts.clear()
    bound_list.clear()
    obs_coords.clear()
    obs_abs_coords.clear()
    obs_ang.clear()



def main():
    global sub1, pub1, sub2

    rospy.init_node('ISS_Node')        
    pub1= rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        
        sub1=rospy.Subscriber("/m2wr/laser/scan", LaserScan, laserScanCallback)
        rospy.sleep(0.1)
        
        sub2=rospy.Subscriber('/odom', Odometry, gen_controls)


        # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
