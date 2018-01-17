#! /usr/bin/env python
# coding=utf-8

# import dependencies
import rospy
import PID

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
        rospy.init_node('maze_nv-171738')
        self.rate = rospy.Rate(20)

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/laserscan', LaserScan, self.laserCallback)
        self.odom = []
        self.laser = []

        # Publisher
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.vel = Twist()

        # Settings
        self.WALLDISTANCE = 0.3
        self.ERRORMARGIN = 0.000001
        self.ROBOTMOVESPEED = 0.3
        self.ROBOTROTATIONSPEED = 0.2

        # Phases
        self.phase = "SearchApproachWall"
        # Possible values = "left", "right"
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

    def rotateRobotDegree(self, rotateDegree):
        """
        Rotate the robot yaw for the specified degree.
        :param rotateDegree: Angle the robot is rotated.
        :return: void
        """
        # Overflow (underflow) occurs when numbers are outside of inverval [0;360]
        overflow = False
        fluctuationTheshold = 10

        currYawDegree = self.getCurrYawDegree()
        degreeTarget = currYawDegree + rotateDegree

        speed = self.ROBOTROTATIONSPEED

        # Rotate positive degree
        if (rotateDegree > 0):
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

        elif (rotateDegree < 0):
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

    def getMinMiddleDatapoints(self, numberDatapointsFromMiddle):
        """
        Takes the specified amount of range data points from the middle laser scanner data and returns the minimal value.
        :param numberDatapointsFromMiddle: Number of data points to use from the laser sensor middle.
        :return: Smallest range data point in specified interval.
        """
        listMiddleIdx = len(self.laser) / 2
        relevantDatapoints = self.laser[listMiddleIdx - (numberDatapointsFromMiddle / 2): listMiddleIdx + (numberDatapointsFromMiddle / 2)]
        return min(relevantDatapoints)

    def moveStraightInfrontOfWall(self, numberDatapointsFromMiddle):
        """
        Move the robot straight in front of the wall and stop at specified distance.
        :param numberDatapointsFromMiddle: Number of data points to use from the laser sensor middle.
        :return: void
        """
        while (True):
            minDist = self.getMinMiddleDatapoints(numberDatapointsFromMiddle)
            if (minDist <= self.WALLDISTANCE):
                self.vel.linear.x = 0.0
                self.velPub.publish(self.vel)
                return
            else:
                self.vel.linear.x = self.ROBOTMOVESPEED
                self.velPub.publish(self.vel)

    "PHASE 2 - FOLLOW WALL"

    def getEqualsizedDatapointGroups(self, numberDatapoints):
        """
        Slices the laser data into equally big groups. Groupsize is determined by the specified argument.
        :param numberDatapoints: Number of data points in each group. Must divide the laser data without remainder!
        :return: List of sublists for the single groups.
        """
        if (len(self.laser) % numberDatapoints == 0):
            datapointGroups = [self.laser[x:x + numberDatapoints] for x in range(0, len(self.laser), numberDatapoints)]
            return datapointGroups
        else:
            rospy.logerr(
                "getEqualsizedDatapointGroups must be divisible without remainder by its argument numberDatapoints")
            return None

    def calcAvgDist(self, datapoints):
        """
        Calculate the average distance of the data points.
        :param datapoints: Range data points for calculating the average value.
        :return: Average value.
        """
        return sum(datapoints) / float(len(datapoints))

    def allignToWall(self, numberDatapointsFromSide):
        """
        Rotates the robot until it is approximately facing in a 90° angle away from the wall.
        This angle is determined by using the sensor data of the robot sides.
        :param numberDatapointsFromSide: Number of data points to use at the laser sensor side (left/right). 
        :return: void
        """

        def getIdxSmallestAvgDist(datapointGroups):
            """
            Returns the index of the data point group with the smallest average distance from the wall. 
            :param datapointGroups: List of sublists for the single groups. 
            :return: Index of the data point group with the smallest average distance.
            """
            # Set it to the biggest possible value
            smallestAvg = max(self.laser)
            smallestAvgGroupIdx = -1
            for currIdx, dpGroup in enumerate(datapointGroups):
                currAvg = self.calcAvgDist(dpGroup)

                if (currAvg < smallestAvg):
                    smallestAvgGroupIdx = currIdx
                    smallestAvg = currAvg
            return smallestAvgGroupIdx

        rotationSpeed = self.ROBOTROTATIONSPEED

        if (self.followWallDirection == "left"):
            # Use sensor data from the right side of the robot to align it for facing to the left
            targetGroupIdx = 0
        elif (self.followWallDirection == "right"):
            # Use sensor data from the left side of the robot to align it for facing to the right.
            # Also change rotation direction.
            targetGroupIdx = len(self.getEqualsizedDatapointGroups(numberDatapointsFromSide)) - 1
            rotationSpeed *= -1
        else:
            rospy.logerr("Incorrect followWallDirection parameter.")

        prevAvg = 0

        while (True):
            equalsizedDatapointGroups = self.getEqualsizedDatapointGroups(numberDatapointsFromSide)
            avgDist = self.calcAvgDist(equalsizedDatapointGroups[targetGroupIdx])

            if (getIdxSmallestAvgDist(equalsizedDatapointGroups) == targetGroupIdx and prevAvg > avgDist):
                # Add some delay for fine adjustment
                rospy.sleep(rospy.Duration(1, 0))
                break
            prevAvg = avgDist

            self.vel.angular.z = rotationSpeed
            self.velPub.publish(self.vel)
            self.rate.sleep()
        self.vel.angular.z = 0.0
        self.velPub.publish(self.vel)

    def followWall(self, numberDatapointsFromMiddle, numberDatapointsFromSide):
        """
        Enables the robot to follow the wall.
        The distance from the robot side to the wall is measured and compared with the target distance.
        A PID controller ensures that this distance can be adhered to with relatively small fluctuation.
        If a wall is detected in front of the robot, it will rotate for 90° to the specified direction.
        :param numberDatapointsFromMiddle: Number of data points to use from the laser sensor middle.
        :param numberDatapointsFromSide: Number of data points to use at the laser sensor side (left/right). 
        :return: void
        """

        # Init PID
        # previously best tested Values #1: P=0.5, I=0.0, D=1.6
        # previously best tested Values #2: P=0.6, I=0.0, D=1.4
        P = 0.6
        I = 0
        D = 1.4
        pid = PID.PID(P, I, D)
        pid.SetPoint = self.WALLDISTANCE
        pid.setSampleTime(0.0)

        errorValuePrev = 0

        while (True):
            # Calculate the avg distance from the robot side to the wall
            datapointsInGroup = 90
            equalsizedDatapointGroups = self.getEqualsizedDatapointGroups(datapointsInGroup)

            if (self.followWallDirection == "left"):
                # Use sensor data from the right side of the robot to follow the wall to the left
                targetGroupIdx = 0
                followWallDirectionAdjustment = 1
            elif (self.followWallDirection == "right"):
                # Use sensor data from the left side of the robot to follow the wall to the right.
                # Also change rotation direction.
                targetGroupIdx = len(equalsizedDatapointGroups) - 1
                followWallDirectionAdjustment = -1
            else:
                rospy.logerr("Incorrect followWallDirection parameter.")
                return

            minDistWallfollowSide = min(equalsizedDatapointGroups[targetGroupIdx])

            # PID control cycle
            pid.update(minDistWallfollowSide)
            controlVariable = pid.output

            # rospy.loginfo("ERRORDIFF: " + str(errorValue - errorValuePrev))
            # errorValuePrev = errorValue

            self.vel.linear.x = self.ROBOTMOVESPEED
            # Applying the PID output as robot rotation
            self.vel.angular.z = controlVariable * followWallDirectionAdjustment

            # Check if wall is in front of robot
            minMiddleDatapoints = self.getMinMiddleDatapoints(numberDatapointsFromMiddle)
            if (minMiddleDatapoints <= self.WALLDISTANCE):
                # Turn the robot 90° if wall is in front
                self.vel.linear.x = 0
                self.velPub.publish(self.vel)

                self.allignToWall(numberDatapointsFromSide)

            self.velPub.publish(self.vel)
            self.rate.sleep()

    "MAIN METHOD"

    def startRobot(self):
        """
        Main method for controlling the robot.
        :return: True if finished successfully or False if not.
        """
        rospy.loginfo("Start!")

        self.phase = "SearchApproachWall"
        while not (self.laser and self.odom):
            # Required because these data are provided with some delay.
            rospy.loginfo("Wait for laser and odom data..")

        while not rospy.is_shutdown():
            if (self.phase == "SearchApproachWall"):
                rospy.loginfo("Phase: SearchApproachWall")

                # Rotate robot to estimated best direction
                datapointGroups = self.classifyDatapoints()
                absoluteIdxBestDatapoint = self.getAbsoluteIdxBestDatapoint(datapointGroups)
                rotDegree = self.calcRotationForBestDatapoint(absoluteIdxBestDatapoint)
                self.rotateRobotDegree(rotDegree)

                # Move robot straight forward
                numberDatapointsFromMiddle = 10
                self.moveStraightInfrontOfWall(numberDatapointsFromMiddle)

                self.phase = "FollowWall"

            elif (self.phase == "FollowWall"):
                rospy.loginfo("Phase: FollowWall")

                numberDatapointsFromMiddle = 180
                numberDatapointsFromSide = 10
                self.followWall(numberDatapointsFromMiddle, numberDatapointsFromSide)

                self.phase = "Finish"

            elif (self.phase == "Finish"):
                rospy.loginfo("Finish")
                return True

            else:
                rospy.logerr("Phasename does not match any defined phase.")
                return False

            # Sleep to prevent flooding your console
            self.rate.sleep()

        rospy.spin()


"MAIN FUNCTION"

if __name__ == "__main__":
    # instantiates your class and calls the __init__ function
    robot = Robot()

    robot.startRobot()
