#!/home/atharva/BTP/btp/bin/python

import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
import time
sys.path.append('../../')
import do_mpc

from std_srvs.srv import Empty
from numpy.lib.function_base import vectorize
import rospy
from std_msgs.msg import String
import numpy as np
import std_msgs.msg
import math
#import tf
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
import message_filters
import settings



from template_model import template_model
from template_mpc import template_mpc
from template_simulator import template_simulator

def odometry_callback(*args):
    global Odom
    Odom = []
    n_agents = len(args)
    for i in range(n_agents):
        Odom.append(args[i])
    # print('got_callback')

def get_state():
    # global OdomStruct
    # OdomStruct = []
    for id in range(6):
        OdomData = Odom[id]
        W = OdomData.pose.pose.orientation.w
        X = OdomData.pose.pose.orientation.x
        Y = OdomData.pose.pose.orientation.y
        Z = OdomData.pose.pose.orientation.z

        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)

        x0 = np.zeros((12,1))
        # print(x0.shape)

        x0[0] = OdomData.pose.pose.position.x
        x0[1] = OdomData.pose.pose.position.y
        x0[2] = OdomData.pose.pose.position.z
        x0[3] = OdomData.twist.twist.linear.x
        x0[4] = OdomData.twist.twist.linear.y
        x0[5] = OdomData.twist.twist.linear.z
        x0[6] = phi
        x0[7] = theta
        x0[8] = psi
        x0[9] = OdomData.twist.twist.angular.x
        x0[10] = OdomData.twist.twist.angular.y
        x0[11] = OdomData.twist.twist.angular.z
        settings.OdomStruct[id] = x0 
        # settings.OdomStruct.append(x0)
    # print(OdomStruct)
    # x0 = np.ones((12,1))

    return x0

def get_u(p_a, p_b, v_a, v_b, r_a, r_b, tau):
    # print('0')
    # print('in get_u')
    # print(norm(p_b - p_a))
    # if norm(p_b - p_a) < 1.2: 
    #     p_b = p_b + 0.2*norm(p_b - p_a)
    #     print('in exception')
    #     print(norm(p_b - p_a))
    m = (p_b-p_a)
    m = m/norm(m)
    # print('1')
    v_rel = v_a - v_b
    theta_m = atan2(m[1],m[0])
    # print(theta_m)
    # print(norm(r_b+r_a)/norm(p_b-p_a))
    theta_i = asin(norm(r_b+r_a)/norm(p_b-p_a))
    plus_line_m = tan(theta_m+theta_i)
    minus_line_m = tan(theta_m-theta_i)
    lb = cos(pi/2 + theta_i)
    v_rel_p = (v_a-v_b)-(p_b-p_a)/tau
    # print('inside get_u')
    # print((p_a, p_b, v_a, v_b, r_a, r_b, tau))
    # print(np.dot(m.T,v_rel_p/norm(v_rel_p))[0][0])
    # print(lb)
    if np.dot(m.T,v_rel_p/norm(v_rel_p))[0][0] < lb:
        n = (-(v_a-v_b)+(p_b-p_a)/tau)
        mag = norm(n)-((r_a+r_b)/tau)
        if mag < 0:
            sign = -1
        else: 
            sign = 1
        n = n/norm(n)
        # print((r_a+r_b)/tau)
        # print(n)
        # print(mag)
        u = mag*n
        # print('circle!')

    else:
        u1_plus = np.array([[1],[plus_line_m]])
        u1_minus = np.array([[1],[minus_line_m]])
        # print(v_rel.shape)
        # print(u1_plus.shape)
        p_hat_plus = (np.dot(v_rel.T,u1_plus)/np.dot(u1_plus.T,u1_plus))*u1_plus
        p_hat_minus = (np.dot(v_rel.T,u1_minus)/np.dot(u1_minus.T,u1_minus))*u1_minus

        u_plus = p_hat_plus - v_rel
        u_minus = p_hat_minus - v_rel

        if np.dot(m.T,v_rel/norm(v_rel))[0][0] > cos(theta_i):
            sign = -1
        else:
            sign = 1

        if norm(u_plus) < norm(u_minus):
            u = u_plus
            # print('plus')
        else:
            u = u_minus
            # print('minus')

    n = u/norm(u)

    return (u, n, sign) # +1 for outside, -1 for inside

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 


def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



def mpc(my_ID, n_agents, final_pos):
    # global OdomStruct
    # OdomStruct = []
    settings.init()
    for id in range(n_agents):
        x0 = np.ones((12,1))
        x0 = (id+1)*x0
        settings.OdomStruct.append(x0)

    rospy.init_node('talker'+str(my_ID), anonymous=True)
    sub_list = []
    for i in  range(n_agents):
        topic = '/firefly'+str(i+1)+'/odometry_sensor1/odometry'
        sub = message_filters.Subscriber(topic, Odometry)
        sub_list.append(sub)

    ts = message_filters.TimeSynchronizer(sub_list, 10)
    ts.registerCallback(odometry_callback)

    # rospy.Subscriber('/firefly'+str(my_ID+1)+'/odometry_sensor1/odometry', Odometry, odometry_callback)
    
    """ User settings: """
    show_animation = False
    store_results = True

    """
    Get configured do-mpc modules:
    """
    time.sleep(1)
    x0 = settings.OdomStruct[my_ID]
    # print(x0)
    time.sleep(1)
    model = template_model(my_ID, n_agents)
    mpc = template_mpc(model, my_ID, n_agents, final_pos)
    # simulator = template_simulator(model,my_ID, n_agents, final_pos, OdomStruct)
    # estimator = do_mpc.estimator.StateFeedback(model)



    """
    Set initial state
    """
    np.random.seed(99)

    e = np.zeros([model.n_x,1])
    #e[2] = 5
    x0 = e # Values between +3 and +3 for all states

    time.sleep(10)
    x0 = settings.OdomStruct[my_ID]
    mpc.x0 = x0
    # simulator.x0 = x0
    # estimator.x0 = x0
    u0 = 0*np.ones((4,1))
    mpc.u0['u'] = u0
    # simulator.u0['u'] = u0
    # estimator.u0['u'] = u0
    
    
    # Use initial state to set the initial guess.
    mpc.set_initial_guess()

    """
    Setup graphic:
    """

    fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data)
    plt.ion()
    
    

    # """
    # Run MPC main loop:
    # """

    # for k in range(200):
    #     u0 = mpc.make_step(x0)
    #     y_next = simulator.make_step(u0)
    #     x0 = estimator.make_step(y_next)
    #     print(x0[2])

    #     if show_animation:
    #         graphics.plot_results(t_ind=k)
    #         graphics.plot_predictions(t_ind=k)
    #         graphics.reset_axes()
    #         plt.show()
    #         plt.pause(0.01)

    # input('Press any key to exit.')
    
    """
    ROS Subscriber and Publisher 
    """
    
    rate = rospy.Rate(50) # 10hz
    pub_topic = '/firefly' + str(my_ID+1) + '/command/trajectory'
    firefly_command_publisher = rospy.Publisher(pub_topic, MultiDOFJointTrajectory, queue_size=10)
    #rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, odometry_callback)


    traj = MultiDOFJointTrajectory()

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time()
    header.frame_id = 'frame'
    traj.joint_names.append('base_link')
    traj.header=header
    transforms =Transform()
    velocities =Twist()
    accelerations=Twist()
    
    x_log = np.zeros((3,1200))
        
    """
    Run MPC ROS Loop
    """
    k = 0
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    while not rospy.is_shutdown():
        
        # unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # rospy.sleep(0.01)
        # pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        
        #print(OdomData)
        get_state()
        OdomData = settings.OdomStruct[my_ID]
        
        # print(str(my_ID),OdomData)
        start_time = rospy.Time.now()

        x0[0] = OdomData[0]
        x0[1] = OdomData[1]
        x0[2] = OdomData[2]
        x0[3] = OdomData[3]
        x0[4] = OdomData[4]
        x0[5] = OdomData[5]
        x0[6] = OdomData[6]
        x0[7] = OdomData[7]
        x0[8] = OdomData[8]
        x0[9] = OdomData[9]
        x0[10] = OdomData[10]
        x0[11] = OdomData[11]
        
        # W = OdomData.pose.pose.orientation.w
        # X = OdomData.pose.pose.orientation.x
        # Y = OdomData.pose.pose.orientation.y
        # Z = OdomData.pose.pose.orientation.z
        
        # phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        # phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        
        # x0[0] = OdomData.pose.pose.position.x
        # x0[1] = OdomData.pose.pose.position.y
        # x0[2] = OdomData.pose.pose.position.z
        # x0[3] = OdomData.twist.twist.linear.x
        # x0[4] = OdomData.twist.twist.linear.y
        # x0[5] = OdomData.twist.twist.linear.z
        # x0[6] = phi
        # x0[7] = theta
        # x0[8] = psi
        # x0[9] = OdomData.twist.twist.angular.x
        # x0[10] = OdomData.twist.twist.angular.y
        # x0[11] = OdomData.twist.twist.angular.z
        # print('x'+str(my_ID))
        # print(x0.T)
    
        
        
        
        u0 = mpc.make_step(x0)
        # print('u'+str(my_ID))
        # print(u0.T)
        
        mpc.x0 = x0
        # simulator.x0 = x0
        # estimator.x0 = x0
        mpc.u0 = u0
        # simulator.u0 = u0
        # estimator.u0 = u0
        mpc.set_initial_guess()
        
    
        # which simualation
        # y_next = simulator.make_step(u0)
        # x0 = estimator.make_step(y_next)
        
        if show_animation:
            graphics.plot_results(t_ind=k)
            graphics.plot_predictions(t_ind=k)
            graphics.reset_axes()
            plt.show()
            plt.pause(0.01)
            
            
            
        
        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[0]
        
        accelerations.angular.x = u0[1]
        accelerations.angular.y = u0[2]
        accelerations.angular.z = u0[3]
        
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        

        print(k)

        traj.points.append(point)
        firefly_command_publisher.publish(traj) 

        x_log[0,k] = x0[0]
        x_log[1,k] = x0[1]
        x_log[2,k] = x0[2]
        #rate.sleep()
        k = k+1
   
        end_time = rospy.Time.now()
        time_taken = (end_time - start_time)/(1e9)
        # if(my_ID==1):
        #     print("time_taken: ",time_taken)
        #rate.sleep()
        #rospy.sleep()
        if k == 1200:
            break
        
        

            
    
    # Store results:
    # if store_results:
    #     do_mpc.data.save_results([mpc, simulator], 'oscillating_masses')
        
    np.save('rec'+str(my_ID), x_log)

if __name__ == '__main__':
    try:
        #rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, odometry_callback)
        mpc(0,6,[4,-6,3])
    except rospy.ROSInterruptException:
        pass
