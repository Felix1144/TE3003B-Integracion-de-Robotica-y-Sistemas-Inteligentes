#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

#Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0
a = l/2
J = (4/3)*m*(a*a)

#Define the callback functions
def simulate_dynamic():
    global J,Tau,m,g,a,x1,x2
    x2_dot =(1/J)*(Tau-(m*g*a)*np.cos(x1)-k*x2)

    x2 += x2_dot*(1/100.0)
    x1 += x2*(1.0/100.0)
    x1 = wrap_to_Pi(x1)

    #Configuración del Header
    header = Header()
    header.frame_id = 'q'
    header.stamp = rospy.Time.now()

    #Define el JointState
    msg = JointState()
    msg.header = header
    msg.position =[x1]
    msg.velocity =[x2]
    msg.name = ['joint2']
    msg.effort = [0.0]

    return msg

#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

def tau_callback(data):
    global Tau
    rospy.loginfo("Mensaje recibido: %s",data.data)
    Tau = data.data

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   
    msg = JointState()
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers
    tau_sub = rospy.Subscriber("/tau", Float32, tau_callback)
    #Setup de publishers
    joint_publisher = rospy.Publisher('/joint_states',JointState,queue_size=10)

    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        while not rospy.is_shutdown():
            msg = simulate_dynamic()
            joint_publisher.publish(msg)
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
