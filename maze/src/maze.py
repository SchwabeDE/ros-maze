#! /usr/bin/env python

# import dependencies
import rospy

# import messages
'''example'''
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Robot:
    # initialise your node, publisher and subscriber as well as some member variables
    def __init__(self):
        rospy.init_node('maze')
        self.rate = rospy.Rate(10)

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCallback)

        # Publisher
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # Member variables
        self.vel = Twist()
        self.odom_x = 0

    def odomCallback(self, data):
        pass
        #TODO implement callback

    # TODO maze runner algorithm
    def startRobot(self):
        rospy.loginfo("start")

        while not rospy.is_shutdown():

            #rospy.loginfo(self.odom_x)

            #self.vel.linear.x = 0.1
            #self.velPub.publish(self.vel)

            # sleep to prevent flooding your console
            self.rate.sleep()

        rospy.spin()


# main function outside your class that is called when the script is executed
if __name__ == "__main__":
    # instantiates your class and calls the __init__ function
    robot = Robot()

    robot.startRobot()
