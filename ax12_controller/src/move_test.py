from moveit_commander import MoveGroupCommander

armGroup = MoveGroupCommander("ax_arm")
armOrigin = [0.0, -0.9, 0.0, 0.0]
gripGroup = MoveGroupCommander("ax_gripper")


def gripperToAngle(angle):
    g = {'gripper_joint': angle}
    gripGroup.set_joint_value_target(g)
    gripGroup.go()


def armToAngles(angles):
    g = {'shoulder_pan_joint': angles[0]}
    g['shoulder_pitch_joint'] = angles[1]
    g['elbow_flex_joint'] = angles[2]
    g['wrist_roll_joint'] = angles[3]
    armGroup.set_joint_value_target(g)
    armGroup.go()


if __name__ == '__main__':
    armToAngles([0.9, 0.2, -0.8, 0.8])
    armToAngles([0.0, 0.0, 0.0, 0.0])
    armToAngles([-0.8, 0.4, 0.6, -0.9])
    armToAngles([0.9, 0.2, -0.8, 0.8])
    armToAngles(armOrigin)
    gripperToAngle(0.1)
    print 'hopefully the arm just moved.'
