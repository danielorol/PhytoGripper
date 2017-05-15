import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates, LinkState
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties, SetJointProperties, GetJointPropertiesRequest, GetJointPropertiesResponse
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Wrench
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math, time
import matplotlib.pyplot as plt

# from std_msgs.srv import Empty
frontUp = False
mouthUp = False
command = False
offset = 0.0
params = 1

def joy_callback(data):
    global frontUp, command, mouthUp, params
    params = 1
    if data.axes[2] > 0:
        mouthUp = True
    else:
        mouthUp = False
    if data.axes[6] > 0:
        frontUp = True
    else:
        frontUp = False
    if data.axes[4] > 0:
        command = True
    else:
        command = False

def cmd_callback(data):
    global frontUp, command, mouthUp, params
    params = 2
    if data.data == 'True':
        command = True
    elif data.data == 'False':
        command = False
    else:
        cmd = data.data
        cmd = cmd.lower()
        try:
            if cmd == "frontup" or cmd == "front up":
                frontUp = True
            elif cmd == "frontdown" or cmd == "front down":
                frontUp = False
            elif cmd == "mouthup" or cmd == "mouth up":
                mouthUp = True
            elif cmd == "mouthdown" or cmd == "mouth down":
                mouthUp = False
            elif cmd =="bothup" or cmd == "both up":
                mouthUp = True
                frontUp = True
            elif cmd == "bothdown" or cmd == "both down":
                mouthUp = False
                frontUp = False
        except:
            print 'Invalid Input'

def cmd_listener():
    rospy.Subscriber("/cmd_publish_mouth",String,cmd_callback)

def joy_listener():
    rospy.init_node('joy_listener')
    rospy.Subscriber("/joy", Joy, joy_callback)


def talker():
    force = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    target_joint_state = rospy.ServiceProxy('/gazebo/set_joint_properties', SetJointProperties)
    hz = 100
    rate = rospy.Rate(hz)  # 100hz
    strength = 4
    while not rospy.is_shutdown():
        mouth_pos = joint_state("phyto::arm::Gripper_arm_joint").position[0]
        mouth_joint = "phyto::arm::Gripper_arm_joint"
        front_pos = joint_state("phyto::arm::Front_arm_joint").position[0]
        front_joint = "phyto::arm::Front_arm_joint"

        if mouthUp:
            mouthEffort = -strength
            print "Mouth Open"
        elif mouth_pos < -0.1:
            mouthEffort = strength
            print "Mouth Closed"
        else:
            mouthEffort = 0
            print "Mouth Closed"
        if frontUp:
            frontEffort = -strength
            print "Front Open"
        elif front_pos < -0.1:
            frontEffort = strength
            print "Front Closed"
        else:
            frontEffort = 0
            print "Front Closed"

        print "Mouth Position = " + str(mouth_pos)
        print "Front Arm Position = " + str(front_pos)
        #print joint_state("phyto::arm::Gripper_arm_joint")
        print command

        if command == True:
            force(mouth_joint,mouthEffort,None,rospy.Duration(1.05 / hz))
            force(front_joint, frontEffort, None, rospy.Duration(1. / hz))
        else:
            force(mouth_joint,0,None,None)
            force(front_joint,0,None,None)
        rate.sleep()


if __name__ == '__main__':
    joy_listener()
    cmd_listener()
    talker()
