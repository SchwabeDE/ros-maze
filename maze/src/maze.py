#! /usr/bin/env python

# import dependencies
import rospy

# import messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Robot:
    # initialise your node, publisher and subscriber as well as some member variables
    def __init__(self):
        rospy.init_node('maze')
        self.rate = rospy.Rate(10)

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/laserscan', LaserScan, self.laserCallback)

        # Publisher
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # Member variables
        self.vel = Twist()
        self.odom = []
        self.laser = []

        self.phase = "SearchApproachWall"
        self.datapointGroups = []
        self.absoluteIdxBestDatapoint = -1

    def odomCallback(self, data):
        '''
        Get relevant odom data (relevant sublists: position(x,y) and orientation(x,y,z,w)).
        :param data: 
        :return: position(x,y) and orientation(x,y,z,w)
        '''
        self.odom = data.pose.pose

    def laserCallback(self, data):
        '''
        Get relevant laserscan data (relevant sublist: ranges).
        :param data: 
        :return: ranges
        '''
        self.laser = data.ranges

    def classifyDatapoints(self, ptpDistanceThreshold = 0.1):

        rangeList = []
        for idx, datapoint in enumerate(self.laser):
            if not (rangeList):
                rangeList.append(datapoint)
            elif(abs(datapoint - rangeList[-1]) <= ptpDistanceThreshold):
                rangeList.append(datapoint)
            else:
                avg = sum(rangeList) / float(len(rangeList))

                #calculate variance
                var = sum(map(lambda x:(x-avg)**2, rangeList))
                #calculate standard deviation
                std = ((1/float(len(rangeList))) * var) ** 0.5

                dpGroup = [rangeList, idx-len(rangeList), avg, std]
                self.datapointGroups.append(dpGroup)
                rangeList = []

    def getAbsoluteIdxBestDatapoint(self, minPointsThreshold=20):

        bestScore = 0
        idxBestScore = 0
        for idx, datapointGroup in enumerate(self.datapointGroups):
            if(len(datapointGroup[0]) >= minPointsThreshold):
                scoreLen = len(datapointGroup[0])/float(len(self.laser))
                scoreAvg = datapointGroup[2]/max(self.laser)
                scoreStd = 1.0 - datapointGroup[3]
                scoreSum = scoreLen + scoreAvg + scoreStd
                #rospy.loginfo("scoreSum: " + str(scoreSum))

                if(scoreSum > bestScore):
                    idxBestScore = idx
                    bestScore = scoreSum
        self.absoluteIdxBestDatapoint = self.datapointGroups[idxBestScore][1] + len(self.datapointGroups[idxBestScore][0])/2

    def calcRotationForBestDatapoint(self):

        return (self.absoluteIdxBestDatapoint - 179) * 0.5


    def startRobot(self):
        rospy.loginfo("start")

        while not rospy.is_shutdown():
            if(self.phase == "SearchApproachWall"):
                while not (self.laser):
                    rospy.loginfo("Wait for laser data..")

                self.classifyDatapoints()
                self.getAbsoluteIdxBestDatapoint()
                rotDegree = self.calcRotationForBestDatapoint()
                #rospy.loginfo(self.datapointGroups[self.idxBestScore])
                #rospy.loginfo("Best score idx:" + str(self.idxBestScore))
                #rospy.loginfo("absoluteIdxBestDatapoint:" + str(self.absoluteIdxBestDatapoint))
                #rospy.loginfo("rotDegree: " + str(rotDegree))
                return


            elif(self.phase == "FollowWall"):
                pass
            else:
                rospy.logerr("Phasename does not match any defined phase.")

            # sleep to prevent flooding your console
            self.rate.sleep()

        rospy.spin()


# main function outside your class that is called when the script is executed
if __name__ == "__main__":
    # instantiates your class and calls the __init__ function
    robot = Robot()

    robot.startRobot()
