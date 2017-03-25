import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/cmd_publish', String, queue_size=10)
    rospy.init_node('cmd_publish', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        msg = input('Enter Desired Angle: ')
        pub.publish(str(msg))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass