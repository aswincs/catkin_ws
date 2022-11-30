#!/usr/bin/env python3
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rospy
import control
import math
import numpy as np

# Got these constants from Gazebo directly
m = 0.029804  
M = 0.021197 
g = 9.8
L = 0.3
r = 0.12

a1 = (1.0/3.0)*M*r*r + m*r*r;
a2 = (1.0/2.0)*m*r*L;
a3 = (1.0/3.0)*m*L*L
a4 = (1.0/2.0)*m*g*L

a23 = a2*a4/(a2*a2 - a1*a3);
a43 = -a1*a4/(a2*a2 - a1*a3)

b21 = -a3/(a2*a2 - a1*a3)
b41 = a2/(a2*a2 - a1*a3)

A = np.matrix(
    [[0, 1, 0, 0],
    [0, 0, a23, 0],
    [0, 0, 0, 1],
    [0, 0, a43, 0]]
)

B = np.matrix(
    [[0],
    [b21],
    [0],
    [b41]]
)

Q = np.diag([1, 10, 20, 1])

R = np.matrix([500])

# Use control module to get optimal gains
K, S, E = control.lqr( A, B, Q, R )

class LQR_InvertedPendulum_Controller:
    def __init__(self):
        rospy.init_node('LQR_Controller')
        # Publishing to joint1 command vel to move cart easier
        self.command_pub = rospy.Publisher("/robo/joint1_effort_controller/command",
                                            Float64, queue_size=10)
        # Subscribing to two topics to get angular pos/vel and linear pos/vel of pendulum
        self.theta_sub = rospy.Subscriber("/robo/joint2_position_controller/state",
                                          JointControllerState, self.theta_callback)
        self.pos_sub = rospy.Subscriber("/robo/joint_states",
                                        JointState, self.pos_callback)
        self.current_state = np.array([0., 0., 0., 0.])
        self.desired_state =                                                              np.array([0., 0., 0., 0.])
        self.command_msg = Float64()
    
    def theta_callback(self, theta_msg):
        # Callback to update angular state variables
        self.current_state[2] = theta_msg.process_value
        self.current_state[3] = theta_msg.process_value_dot
        rospy.loginfo_throttle(2, f'Current Angle: {math.degrees(theta_msg.process_value)}')
        
    def pos_callback(self, pos_msg):
        # Callback to update linear state variables
        self.current_state[0] = pos_msg.position[1]
        self.current_state[1] = pos_msg.velocity[1]
        
    def balance(self):
        # Get control output by multiplying K with state error
        self.command_msg.data = np.matmul(K, (self.desired_state - self.current_state))
        #print('THE VALUE OF DESIRED STATE IS ', self.command_msg.data)
        #print('THE VALUE OF CURRENT STATE IS ', self.current_state)
        #for i in range(100):
        #    if i%2==0:
        #        a = 0.05
        #    else:
        #        a = -0.05
        #    #a = 0.01
        #    self.command_msg = a
        self.command_pub.publish(self.command_msg)
        rospy.loginfo_throttle(2, f'Commanding: {self.command_msg}')

def main():
    b = LQR_InvertedPendulum_Controller()
    while not rospy.is_shutdown():
        b.balance()          

if __name__ == '__main__':
    main()
