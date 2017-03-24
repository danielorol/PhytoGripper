import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates, LinkState
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties, SetJointProperties, GetJointPropertiesRequest, GetJointPropertiesResponse
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Wrench
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Joy
import math, time

# from std_msgs.srv import Empty
des_joint_pos = 0.0
command = False
offset = 0.0

def callback(data):
    global des_joint_pos, command, offset
    des_joint_pos = -(data.axes[2]+1)/2.0*math.pi
    offset = data.axes[2]*math.pi/2.0
    if data.axes[4] > 0:
        command = True
    else:
        command = False


def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/joy", Joy, callback)


def talker():
    # rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    force = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    target_joint_state = rospy.ServiceProxy('/gazebo/set_joint_properties', SetJointProperties)
    hz = 100
    rate = rospy.Rate(hz)  # 100hz
    error = 0
    lastError = 0
    t = time.clock()
    k = 1
    k2 = -0.3
    kd = 0.1
    while not rospy.is_shutdown():
        t2 = time.clock()
        joint_pos = joint_state("PhytobiopsyAssembly::Arm_joint").position[0]
        joint_name = "PhytobiopsyAssembly::Arm_joint"
        error = des_joint_pos-joint_pos
        errorRate = (error-lastError)/(t2-t)
        effort = k*error+k2*offset+kd*errorRate
        #print effort
        if command == True:
            force(joint_name,effort,None,rospy.Duration(1.0/hz))
        else:
            force(joint_name,0,None,None)
        lastError=error
        t = t2
        rate.sleep()


if __name__ == '__main__':
    listener()
    talker()