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

    robot1ID = p.loadURDF("kuka_iiwa/model.urdf", [0, 2, 0], [0, 0, 0, 1], globalScaling=2)
    robot2ID = p.loadURDF("kuka_iiwa/model.urdf", [0, -2, 0], [0, 0, 0, 1], globalScaling=2)

    belt1 = p.loadURDF("E:/Academic/Master/RL/pybullet_robot_planning/urdf/conveyor_belt.urdf")
    p.resetBasePositionAndOrientation(belt1, [0, 2, 0], [0, 0, 0, 1])
    belt2 = p.loadURDF("E:/Academic/Master/RL/pybullet_robot_planning/urdf/conveyor_belt.urdf")
    p.resetBasePositionAndOrientation(belt2, [0, -2, 0], [0, 0, 0, 1])

    constraint1ID = p.createConstraint(belt1, -1, robot1ID, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1])
    constraint2ID = p.createConstraint(belt2, -1, robot2ID, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1])

    numJoints = p.getNumJoints(robot1ID)
    print("numJoints:", numJoints)
    jointPositions = [p.getJointState(robot1ID, i)[0] for i in range(numJoints)]
    print("jointPositions:", jointPositions)

    endEffectorName = "lbr_iiwa_link_7"
    endEffectorIndex = -1
    for jointIndex in range(numJoints):
        jointInfo = p.getJointInfo(robot1ID, jointIndex)
        if jointInfo[12].decode("utf-8") == endEffectorName:
            endEffectorIndex = jointIndex
            break

    if endEffectorName == -1:
        print("End effector not found")
    else:
        print("End effector index:", endEffectorIndex)

    # robot1
    targetPositions1 = [0, -2, 2]
    print("targetPositions1:", targetPositions1)
    targetOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
    print("targetOrientation1:", targetOrientation1)
    targetJointPositions1 = p.calculateInverseKinematics(robot1ID, endEffectorIndex, targetPositions1, targetOrientation1)
    print("targetJointPositions:", targetJointPositions1)
    p.setJointMotorControlArray(robot1ID, range(numJoints), p.POSITION_CONTROL, targetJointPositions1)

    # robot2
    targetPositions2 = [0, 2, 2]
    print("targetPositions2:", targetPositions2)
    targetOrientation2 = p.getQuaternionFromEuler([0, 0, 0])
    print("targetOrientation2:", targetOrientation2)
    targetJointPositions2 = p.calculateInverseKinematics(robot2ID, endEffectorIndex, targetPositions2,
                                                         targetOrientation2)
    print("targetJointPositions2:", targetJointPositions2)
    p.setJointMotorControlArray(robot2ID, range(numJoints), p.POSITION_CONTROL, targetJointPositions2)

    for i in range(100):
        p.stepSimulation()
        time.sleep(1./10.)
        print("i:", i)

    # close server
    p.disconnect()