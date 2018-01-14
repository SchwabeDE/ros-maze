#! /usr/bin/env python
# coding=utf-8

# import dependencies
import rospy

# import messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Robot:
    """
    Main class for controlling the robot.
    """

    def __init__(self):
        rospy.init_node('maze')
        self.rate = rospy.Rate(20)

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/laserscan', LaserScan, self.laserCallback)

        # Publisher
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # Subscriber + Publisher variables
        self.vel = Twist()
        self.odom = []
        self.laser = []

        # Settings
        self.WALLDISTANCE = 0.5
        self.ERRORMARGIN = 0.000001
        self.MOVESPEED = 0.3
        self.ROTATIONSPEED = 0.2

        # Phases
        self.phase = "SearchApproachWall"
        self.followWallDirection = "left"

    "SUBSCRIBER CALLBACKS"

    def odomCallback(self, data):
        """
        Get relevant odom data (relevant sublists: position(x,y) and orientation(x,y,z,w)).
        :param data: /odom topic data
        :return: position(x,y) and orientation(x,y,z,w)
        """
        self.odom = data.pose.pose

    def laserCallback(self, data):
        """
        Get relevant laserscan data (relevant sublist: ranges -> array of distances with 360 elements, spanning 180° degree). 
        :param data: /laserscan topic data
        :return: void
        """
        self.laser = data.ranges

    "PHASE 1 - SEARCH AND APPROACH WALL"

    def classifyDatapoints(self, ptpDistanceThreshold=0.1):
        """
        Classify the data points obtained from the laser scanner into sublists containing points which are close to eachother.
        For each sublist, the absolute index of its first entry and also average + standard deviation information is added.
        :param ptpDistanceThreshold: Determines the distance for grouping data points together.
        :return: Array of classified data points together with meta data. 
                 Format: datapointGroups[ datapointGroup[rangeList, relativeIdx, average, standardDeviation] ]
        """
        datapointGroups = []
        rangeList = []
        for idx, datapoint in enumerate(self.laser):
            if not (rangeList):
                rangeList.append(datapoint)
            elif (abs(datapoint - rangeList[-1]) <= ptpDistanceThreshold):
                rangeList.append(datapoint)
            else:
                relIdx = idx - len(rangeList)

                # calculate average(arithmetic mean)
                avg = sum(rangeList) / float(len(rangeList))
                # calculate variance
                var = sum(map(lambda x: (x - avg) ** 2, rangeList))
                # calculate standard deviation
                std = ((1 / float(len(rangeList))) * var) ** 0.5

                dpGroup = [rangeList, relIdx, avg, std]
                datapointGroups.append(dpGroup)
                rangeList = []
        return datapointGroups

    def getAbsoluteIdxBestDatapoint(self, datapointGroups, minPointsThreshold=20):
        """
        Get the absolute index of the middle data point from the most promising data point group.
        The most promising data point group is determined assigning a score to each group.
        The score for each group is calculated regarding to the number of datapoints (higher = better), 
        the average distance (higher = better) and standard deviation (lower = better).
        :param datapointGroups: [ datapointGroup[rangeList, relativeIdx, average, standardDeviation] ]
        :param minPointsThreshold: The minimum size of data points a group must provide.
        :return: Absolute index of the best data point in interval [0;359].
        """
        bestScore = 0
        idxBestScore = 0
        for idx, datapointGroup in enumerate(datapointGroups):
            if (len(datapointGroup[0]) >= minPointsThreshold):
                # number of datapoints and average must be normalized
                scoreLen = len(datapointGroup[0]) / float(len(self.laser))
                scoreAvg = datapointGroup[2] / max(self.laser)
                scoreStd = 1.0 - datapointGroup[3]
                scoreSum = scoreLen + scoreAvg + scoreStd
                # rospy.loginfo("scoreSum: " + str(scoreSum))

                if (scoreSum > bestScore):
                    idxBestScore = idx
                    bestScore = scoreSum
        absoluteIdxBestDatapoint = datapointGroups[idxBestScore][1] + len(
            datapointGroups[idxBestScore][0]) / 2
        return absoluteIdxBestDatapoint

    def calcRotationForBestDatapoint(self, absoluteIdxBestDatapoint):
        """
        Calculate degree the roboter must rotate to face the best data point.
        :param absoluteIdxBestDatapoint: Absolute index of the best data point in interval [0;359].
        :return: Degree in interval [-180;180]
        """
        return (absoluteIdxBestDatapoint - 179) * 0.5

    def getCurrYawDegree(self):
        """
        Converts the odom orientation quaternion data into yaw degree.
        :return: Yaw degree in interval [0;359]
        """
        orientation = self.odom.orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        # rad from quaternion
        (roll, pitch, yaw) = euler_from_quaternion(orientationList)
        # degree from rad
        pi = 3.14159265359
        yawDegree = (180 / pi) * yaw

        # transform [-180;180] degree into [0;359]
        if (yawDegree < 0):
            yawDegree = 359 + yawDegree

        return yawDegree

    def rotateRobotDegree(self, requestTurnDegree, speed):
        """
        Rotate the robot yaw for the specified degree.
        :param requestTurnDegree: 
        :param speed: Turn speed
        :return: void
        """
        # Overflow (underflow) occurs when numbers are outside of inverval [0;360]
        overflow = False
        fluctuationTheshold = 10

        currYawDegree = self.getCurrYawDegree()
        degreeTarget = currYawDegree + requestTurnDegree

        # Rotate positive degree
        if (requestTurnDegree > 0):
            while (currYawDegree < degreeTarget):
                prevYawDegree = currYawDegree
                # Adjust the current degree for overflow error
                if (overflow):
                    currYawDegree = self.getCurrYawDegree() + 359
                else:
                    currYawDegree = self.getCurrYawDegree()

                self.vel.angular.z = speed
                self.velPub.publish(self.vel)

                # Overflow detection
                if (currYawDegree + fluctuationTheshold < prevYawDegree):
                    overflow = True

                    # rospy.loginfo("TURN POS")
                    # rospy.loginfo("currYawDegree: " + str(currYawDegree))
                    # rospy.loginfo("degreeTarget: " + str(degreeTarget))

        elif (requestTurnDegree < 0):
            while (currYawDegree > degreeTarget):
                prevYawDegree = currYawDegree
                # Adjust the current degree for overflow error
                if (overflow):
                    currYawDegree = self.getCurrYawDegree() - 360
                else:
                    currYawDegree = self.getCurrYawDegree()

                self.vel.angular.z = speed * -1
                self.velPub.publish(self.vel)

                # Overflow detection
                if (currYawDegree - fluctuationTheshold > prevYawDegree):
                    overflow = True
        else:
            # nothing to do
            # rospy.loginfo("Nothing to do")
            return

        self.vel.angular.z = 0.0
        self.velPub.publish(self.vel)

    def getAvgMiddleDatapoints(self, datapoints=10):
        """
        Calculate the average range of the specified amount of data points.
        :return: Average of data points.
        """
        listMiddle = len(self.laser) / 2
        relevantDatapoints = self.laser[listMiddle - (datapoints / 2): listMiddle + (datapoints / 2)]
        return sum(relevantDatapoints) / float(len(relevantDatapoints))

    def moveStraightBeforeWall(self, speed, datapoints=10):
        """
        Move the robot straight in front of the wall and stop at specified distance.
        :param speed: Move speed
        :param datapoints: Number of data points to use.
        :return: void
        """
        while (True):
            avgDist = self.getAvgMiddleDatapoints(datapoints)
            # rospy.loginfo("avgDist: " + str(avgDist))
            if (avgDist <= self.WALLDISTANCE):
                self.vel.linear.x = 0.0
                self.velPub.publish(self.vel)
                return
            else:
                self.vel.linear.x = speed
                self.velPub.publish(self.vel)

    "PHASE 2 - FOLLOW WALL"

    def getEqualsizedDatapointGroups(self, datapoints=10):
        # Put laser scanner data points in euqally large groups
        numberElementsInGroup = len(self.laser) / datapoints
        datapointGroups = []
        for i in range(0, len(self.laser), numberElementsInGroup):
            datapointGroups.append(self.laser[i:i + datapoints])
        return datapointGroups

    def calcAvgDist(self, datapoints):
        return sum(datapoints) / float(len(datapoints))

    def allignToWall(self, datapoints=10, speed=0.2):

        def calcIdxSmallestAvgDist(datapointGroups):
            smallestAvg = max(self.laser)
            smallestAvgGroupIdx = -1
            for currIdx, dpGroup in enumerate(datapointGroups):
                currAvg = self.calcAvgDist(dpGroup)

                if (currAvg < smallestAvg):
                    smallestAvgGroupIdx = currIdx
                    smallestAvg = currAvg
            rospy.loginfo("smallestAvgGroupIdx: " + str(smallestAvgGroupIdx))
            return smallestAvgGroupIdx

        if(self.followWallDirection == "left"):
            targetGroupIdx = 0
        elif(self.followWallDirection == "right"):
            targetGroupIdx = len(self.getEqualsizedDatapointGroups(datapoints))-1
            speed *= -1
        else:
            rospy.logerr("Incorrect followWallDirection parameter.")

        prevAvg = 0

        while (True):
            equalsizedDatapointGroups = self.getEqualsizedDatapointGroups(datapoints)
            avgDist = self.calcAvgDist(equalsizedDatapointGroups[targetGroupIdx])

            if (calcIdxSmallestAvgDist(equalsizedDatapointGroups) == targetGroupIdx and prevAvg > avgDist):
                # Add some delay for fine adjustment
                rospy.sleep(rospy.Duration(1, 0))
                break
            prevAvg = avgDist

            self.vel.angular.z = speed
            self.velPub.publish(self.vel)
            self.rate.sleep()
        self.vel.angular.z = 0.0
        self.velPub.publish(self.vel)

    def followWall(self):

        while(True):

            targetGroupIdx = 0
            equalsizedDatapointGroups = self.getEqualsizedDatapointGroups()
            avgDist = self.calcAvgDist(equalsizedDatapointGroups[targetGroupIdx])


            pvDifference = avgDist - self.WALLDISTANCE
            rospy.loginfo("pvDifference: " + str(pvDifference))

            pvDifferenceTheshold = 0.2
            if(abs(pvDifference) < pvDifferenceTheshold):
                self.vel.linear.x = self.MOVESPEED
            else:
                self.vel.linear.x = 0

            # Too far from wall away
            if(pvDifference > 0):
                self.vel.angular.z = self.ROTATIONSPEED * -1

            # Too close to wall
            elif(pvDifference < 0):
                self.vel.angular.z = self.ROTATIONSPEED

            else:
                self.vel.angular.z = 0.0

            # Wall is in front

            avgMiddleDatapoints = self.getAvgMiddleDatapoints()
            if(avgMiddleDatapoints <= self.WALLDISTANCE):
                self.vel.linear.x = 0
                self.velPub.publish(self.vel)
                self.allignToWall()

            self.velPub.publish(self.vel)
            self.rate.sleep()



    "MAIN METHOD"

    def startRobot(self):
        """
        Main method for controlling the robot.
        :return: void
        """
        rospy.loginfo("Start!")

        self.phase = "SearchApproachWall"
        while not (self.laser and self.odom):
            # Required because these data are provided with some delay.
            rospy.loginfo("Wait for laser and odom data..")

        while not rospy.is_shutdown():
            if (self.phase == "SearchApproachWall"):
                rospy.loginfo("Phase: SearchApproachWall")

                datapointGroups = self.classifyDatapoints()
                absoluteIdxBestDatapoint = self.getAbsoluteIdxBestDatapoint(datapointGroups)
                rotDegree = self.calcRotationForBestDatapoint(absoluteIdxBestDatapoint)
                # rospy.loginfo("rotDegree: " + str(rotDegree))
                # rospy.loginfo("Yaw: " + str(self.getCurrYawDegree()))
                self.rotateRobotDegree(rotDegree, 0.2)
                self.moveStraightBeforeWall(0.3)

                # rospy.loginfo(self.datapointGroups[self.idxBestScore])
                # rospy.loginfo("Best score idx:" + str(self.idxBestScore))
                # rospy.loginfo("absoluteIdxBestDatapoint:" + str(self.absoluteIdxBestDatapoint))
                # rospy.loginfo("rotDegree: " + str(rotDegree))
                self.phase = "FollowWall"

            elif (self.phase == "FollowWall"):
                rospy.loginfo("Phase: FollowWall")

                #self.allignToWall()
                self.followWall()

                self.phase = "Finish"

            elif (self.phase == "Finish"):
                rospy.loginfo("Finish")
                return

            else:
                rospy.logerr("Phasename does not match any defined phase.")

            # sleep to prevent flooding your console
            self.rate.sleep()

        rospy.spin()


"MAIN FUNCTION"

if __name__ == "__main__":
    # instantiates your class and calls the __init__ function
    robot = Robot()

    robot.startRobot()
