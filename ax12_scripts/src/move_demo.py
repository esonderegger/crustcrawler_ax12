#!/usr/bin/env python
# move_demo.py: a demo of some arm movement using MoveIt

from moveit_commander import MoveGroupCommander

armGroup = MoveGroupCommander("ax_arm")
eefGroup = MoveGroupCommander("ax_eef")

homeAngles = [0.0, -0.9, 0.0, 0.0]
leftHigh = [1.0, -0.3, -0.5, 1.0]
cobra = [0.0, -0.95, -1.5, 0.0]
rightHigh = [-1.0, -0.3, -0.5, -1.0]

def moveToAngles(angles):
    g = {'shoulder_pan_joint': angles[0]}
    g['shoulder_pitch_joint'] = angles[1]
    g['elbow_flex_joint'] = angles[2]
    g['wrist_roll_joint'] = angles[3]
    armGroup.set_joint_value_target(g)
    armGroup.go()

def moveGripper(angle):
    g = {'gripper_joint': angle}
    g['r_gripper_aft_joint'] = -angle
    g['l_gripper_fwd_joint'] = angle
    g['r_gripper_fwd_joint'] = -angle
    g['gripper_finger_pincher_joint'] = -angle
    g['r_gripper_aft_pincher_joint'] = -angle
    eefGroup.set_joint_value_target(g)
    eefGroup.go()

if __name__ == '__main__':
    # moveToAngles(homeAngles)
    # moveToAngles(leftHigh)
    # moveToAngles(homeAngles)
    # moveToAngles(rightHigh)
    # moveToAngles(homeAngles)
    moveGripper(0.35)
    moveGripper(-0.10)
    moveGripper(0.25)
    moveGripper(0.0)
