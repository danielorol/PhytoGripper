import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates, LinkState
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties, SetJointProperties, GetJointPropertiesRequest, GetJointPropertiesResponse
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Wrench
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math, time

# from std_msgs.srv import Empty
des_joint_pos = 0.0
command = False
offset = 0.0
params = 1

def joy_callback(data):
    global des_joint_pos, command, offset, params
    params = 1
    des_joint_pos = -(data.axes[2]+1)/2.0*math.pi
    offset = data.axes[2]*math.pi/2.0
    if data.axes[4] > 0:
        command = True
    else:
        command = False

def cmd_callback(data):
    global des_joint_pos, command, offset, params
    params = 2
    if data.data == 'True':
        command = True
    elif data.data == 'False':
        command = False
    else:
        try:
            des_joint_pos = float(data.data)*math.pi/180-math.pi
            #if des_joint_pos > math.pi:
            #    des_joint_pos = math.pi
            #elif des_joint_pos < 0.0:
            #    des_joint_pos = 0.0
            offset = des_joint_pos-math.pi/2
        except:
            print 'Invalid Input'

def cmd_listener():
    rospy.Subscriber("/cmd_publish",String,cmd_callback)

def joy_listener():
    rospy.init_node('joy_listener')
    rospy.Subscriber("/joy", Joy, joy_callback)


def talker():
    force = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    target_joint_state = rospy.ServiceProxy('/gazebo/set_joint_properties', SetJointProperties)
    hz = 100
    rate = rospy.Rate(hz)  # 100hz
    error = 0
    lastError = 0
    lastErrorRate = 0
    t = time.clock()
    while not rospy.is_shutdown():
        if params == 1:
            k = 1
            k2 = -0.3
            kd = 0.15
            alpha = 1
        else:
            k = 1
            k2 = 0
            kd = 0
            alpha = 0.01
        t2 = time.clock()
        joint_pos = joint_state("PhytobiopsyAssembly::Arm_joint").position[0]
        joint_name = "PhytobiopsyAssembly::Arm_joint"
        error = des_joint_pos-joint_pos
        errorRate = alpha*(error-lastError)/(t2-t)*(1-alpha)*lastErrorRate
        effort = k*error+k2*offset+kd*errorRate
        print "Error is " + str(error)
        print "Done"
        #print effort
        if command == True:
            force(joint_name,effort,None,rospy.Duration(0.98/hz))
        else:
            force(joint_name,0,None,None)
        lastError=error
        lastErrorRate = errorRate
        t = t2
        rate.sleep()


if __name__ == '__main__':
    joy_listener()
    cmd_listener()
    talker()
