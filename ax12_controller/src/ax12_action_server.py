#!/usr/bin/env python

import roslib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import GripperCommandAction
from sensor_msgs.msg import JointState
from actionlib import SimpleActionServer
import yaml
import datetime
import lib_dynamixel

roslib.load_manifest('ax12_controller')

ids = [1, 2, 3, 4, 5, 6, 7]
device = '/dev/ttyACM0'
dynamixelChain = lib_dynamixel.Dynamixel_Chain(device, 1000000, ids)


class axPublisher():
    def __init__(self, configJoints, ids):
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState)
        self.jointNames = []
        for configJoint in configJoints:
            self.jointNames.append(configJoint['name'])
            if 'mimic_joints' in configJoint:
                for mimic in configJoint['mimic_joints']:
                    self.jointNames.append(mimic['name'])
        self.ids = ids
        self.jPositions = []
        self.jVelocities = []

    def idValsToJoints(self, idVals):
        # this is ugly and way too specific for the ax-12 arm
        out = [idVals[0]]
        out.append(idVals[1])
        out.append(idVals[3])
        out.append(idVals[5])
        out.append(idVals[6])
        out.append(-idVals[6])
        out.append(idVals[6])
        out.append(-idVals[6])
        out.append(-idVals[6])
        out.append(-idVals[6])
        return out

    def publishJointStates(self):
        jState = JointState()
        jState.header.stamp = rospy.Time.now()
        jState.name = self.jointNames
        try:
            angs, angvs, myids = dynamixelChain.read_angs_angvels(self.ids)
            self.jPositions = self.idValsToJoints(angs)
            self.jVelocities = self.idValsToJoints(angvs)
        except:
            print 'There was a problem reading from the servos.'
            for servoID in self.ids:
                if dynamixelChain.is_eeprom_locked(servoID):
                    print 'servo ' + str(servoID) + ' is locked'
                else:
                    print 'servo ' + str(servoID) + ' is not locked'
            rospy.sleep(0.5)
        jState.position = self.jPositions
        jState.velocity = self.jVelocities
        jState.effort = [0.0] * 10
        self.joint_state_pub.publish(jState)
        return jState


class axServer:
    def __init__(self, name, configJoints):
        self.fullname = name
        self.jointNames = []
        for configJoint in configJoints:
            self.jointNames.append(configJoint['name'])
        self.jointNames = self.jointNames[:4]
        self.failureState = False
        self.goalPositions = [0.0, -0.9, 0.0, 0.0]
        self.goalVelocities = [0.5] * 4
        self.move_chain()
        # todo: add method for checking arm's actual positions
        self.actualPositions = [0.0, -0.9, 0.0, 0.0]
        self.server = SimpleActionServer(self.fullname,
                                         FollowJointTrajectoryAction,
                                         execute_cb=self.execute_cb,
                                         auto_start=False)
        self.server.start()

    def move_chain(self):
        # moves the Dynamixel Chain to the object's goalPositions
        # and goalVelocities
        orderedIDs = [[1], [2, 3], [4, 5], [6]]
        commandIDs = []
        commandPositions = []
        commandVelocities = []
        for i in range(len(self.goalPositions)):
            for orderedID in orderedIDs[i]:
                commandIDs.append(orderedID)
                commandPositions.append(self.goalPositions[i])
                commandVelocities.append(self.goalVelocities[i])
        dynamixelChain.move_angles_sync(commandIDs,
                                        commandPositions,
                                        commandVelocities)

    def execute_cb(self, goal):
        startTime = rospy.Time.now().to_sec()
        rospy.loginfo(goal)
        jNames = goal.trajectory.joint_names
        pointsQueue = goal.trajectory.points
        for a in range(len(pointsQueue)):
            pointTime = pointsQueue[a].time_from_start.to_sec()
            timeSlept = 0.0
            while rospy.Time.now().to_sec() - startTime < pointTime:
                rospy.sleep(0.01)
                timeSlept += 0.01
            print 'slept ' + str(timeSlept) + ' seconds between points'
            rospy.loginfo(rospy.Time.now().to_sec() - startTime)
            if a == 0:
                self.executePoint(pointsQueue[a],
                                  pointsQueue[a], jNames)
            else:
                self.executePoint(pointsQueue[a - 1],
                                  pointsQueue[a], jNames)
        self.server.set_succeeded()

    def executePoint(self, point1, point2, jNames):
        # moves arm to point2's positions at the greater
        # absolute Velocites of the two points
        startTime = datetime.datetime.now()
        for x in range(len(jNames)):
            if jNames[x] in self.jointNames:
                y = self.jointNames.index(jNames[x])
                self.goalPositions[y] = point2.positions[x]
                if abs(point1.velocities[x]) > abs(point2.velocities[x]):
                    self.goalVelocities[y] = abs(point1.velocities[x])
                else:
                    self.goalVelocities[y] = abs(point2.velocities[x])
            else:
                print 'we have a problem: joint names are not lining up'
        self.move_chain()
        endTime = datetime.datetime.now()
        print 'executePoint took: ' + str(endTime - startTime)

    def checkFailureState(self):
        if self.failureState:
            print 'I am currently in a failure state.'


class axGripperServer:
    def __init__(self, name):
        self.fullname = name
        self.goalAngle = 0.0
        self.actualAngle = 0.0
        self.failureState = False
        dynamixelChain.move_angles_sync(ids[6:], [self.goalAngle], [0.5])
        self.server = SimpleActionServer(self.fullname,
                                         GripperCommandAction,
                                         execute_cb=self.execute_cb,
                                         auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo(goal)
        self.currentAngle = goal.command.position
        dynamixelChain.move_angles_sync(ids[6:], [self.currentAngle], [0.5])
        attempts = 0
        for i in range(10):
            rospy.sleep(0.1)
            attempts += 1
        # while ... todo: add some condition to check on the actual angle
        #     rospy.sleep(0.1)
        #     print jPositions[4]
        #     attempts += 1
        if attempts < 20:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

    def checkFailureState(self):
        if self.failureState:
            print 'I am currently in a failure state.'


def axserver():
    rospy.init_node('ax12_action_server')
    ax12Dir = roslib.packages.get_pkg_dir('ax12_controller')
    configFile = open(ax12Dir + '/config/ax12_arm_configuration.yaml')
    configData = yaml.load(configFile)
    configJoints = configData['joints']
    jStatePub = axPublisher(configJoints, ids)
    armServer = axServer('ax12_arm/follow_joint_trajectory', configJoints)
    gripperServer = axGripperServer('ax12_gripper/gripper_command')
    while not rospy.is_shutdown():
        jStatePub.publishJointStates()
        # right now, these two methods exist to keep flake8 from giving
        # errors, although I suppose error checking wouldn't be a bad idea.
        armServer.checkFailureState()
        gripperServer.checkFailureState()
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        axserver()
    except rospy.ROSInterruptException:
        pass
