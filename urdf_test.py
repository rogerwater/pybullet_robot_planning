# Test the urdf file of kuka robot with prismatic joint

import pybullet as p
import pybullet_data
import time
import numpy as np


# 12, 2, 0.25
if __name__ == "__main__":
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    planeID = p.loadURDF("plane.urdf")

    table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[6, 1.2, 0.15])
    table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[6, 1.2, 0.15], rgbaColor=[1, 1, 1, 1])
    tableID = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision, baseVisualShapeIndex=table_visual, basePosition=[0, 0, 0])

    cubeID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[6, 1, 0.125])
    cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[6, 1, 0.125])
    cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[6, 1, 0.125], rgbaColor=[1, 0, 0, 1])
    cubeID = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cube_collision, baseVisualShapeIndex=cube_visual, basePosition=[0, 0, 0.3])

    robotID = p.loadURDF("kuka_iiwa/model.urdf", [0, 2, 0], [0, 0, 0, 1], globalScaling=2)

    numJoints = p.getNumJoints(robotID)
    print("numJoints:", numJoints)
    jointPositions = [p.getJointState(robotID, i)[0] for i in range(numJoints)]
    print("jointPositions:", jointPositions)

    endEffectorName = "lbr_iiwa_link_7"
    endEffectorIndex = -1
    for jointIndex in range(numJoints):
        jointInfo = p.getJointInfo(robotID, jointIndex)
        if jointInfo[12].decode("utf-8") == endEffectorName:
            endEffectorIndex = jointIndex
            break

    if endEffectorName == -1:
        print("End effector not found")
    else:
        print("End effector index:", endEffectorIndex)

    targetPositions = [0, -2, 2]
    print("targetPositions:", targetPositions)
    targetOrientation = p.getQuaternionFromEuler([0, 0, 0])
    print("targetOrientation:", targetOrientation)
    targetJointPositions = p.calculateInverseKinematics(robotID, endEffectorIndex, targetPositions, targetOrientation)
    print("targetJointPositions:", targetJointPositions)
    p.setJointMotorControlArray(robotID, range(numJoints), p.POSITION_CONTROL, targetJointPositions)

    for i in range(100):
        p.stepSimulation()
        time.sleep(1./10.)
        print("i:", i)

    # close server
    p.disconnect()