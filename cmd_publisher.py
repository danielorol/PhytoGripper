import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/cmd_publish', String, queue_size=10)
    rospy.init_node('cmd_publish', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    numInt = 50
    command = 90.
    lastCommand = 90.
    while not rospy.is_shutdown():
        msg = input('Enter Desired Angle: ')
        if type(msg) is float or type(msg) is int:
            lastCommand = command
            command = msg
            delta = command-lastCommand
            interval = delta/numInt
            for i in range(1,numInt+1):
                pub.publish(str(lastCommand+interval*i))
        pub.publish(str(msg))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass