import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import imageio


# 12, 2, 0.25
if __name__ == "__main__":
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    planeID = p.loadURDF("plane.urdf")

    table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[6, 1.2, 0.15])
    table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[6, 1.2, 0.15], rgbaColor=[1, 1, 1, 1])
    tableID = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision, baseVisualShapeIndex=table_visual, basePosition=[0, 0, 0])

    cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[6, 1, 0.125])
    cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[6, 1, 0.125], rgbaColor=[1, 0, 0, 1])
    cubeID = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cube_collision, baseVisualShapeIndex=cube_visual, basePosition=[0, 0, 0.3])

    # robot1ID = p.loadURDF("kuka_iiwa/model.urdf", [0, 2, 0], [0, 0, 0, 1], globalScaling=2)
    # robot2ID = p.loadURDF("kuka_iiwa/model.urdf", [0, -2, 0], [0, 0, 0, 1], globalScaling=2)
    robot1ID = p.loadURDF("urdf/kuka_with_slide/kuka.urdf", [0, 2, 0], [0, 0, 0, 1], globalScaling=2)
    robot2ID = p.loadURDF("urdf/kuka_with_slide/kuka.urdf", [0, -2, 0], [0, 0, 0, 1], globalScaling=2)

    numJoints = p.getNumJoints(robot1ID)
    print("numJoints:", numJoints)
    jointPositions = [p.getJointState(robot1ID, i)[0] for i in range(numJoints)]
    print("jointPositions:", jointPositions)

    # find end effector index
    endEffectorName = "lbr_iiwa_link_7"
    endEffectorIndex = -1
    for jointIndex in range(numJoints):
        jointInfo = p.getJointInfo(robot1ID, jointIndex)
        if jointInfo[12].decode("utf-8") == endEffectorName:
            endEffectorIndex = jointIndex
            break
    if endEffectorIndex == -1:
        print("End effector not found")
    else:
        print("End effector index:", endEffectorIndex)

    # find sliding joint index
    slidingJointName = "sliding_joint"
    slidingJointIndex = -1
    for jointIndex in range(numJoints):
        jointInfo = p.getJointInfo(robot1ID, jointIndex)
        if jointInfo[1].decode("utf-8") == slidingJointName:
            slidingJointIndex = jointIndex
            break
    if slidingJointIndex == -1:
        print("Sliding joint not found")
    else:
        print("Sliding joint index:", slidingJointIndex)

    p.resetJointState(robot1ID, slidingJointIndex, -6)
    p.resetJointState(robot2ID, slidingJointIndex, -6)

    # robot1
    targetPositions1 = [0, -2, 2]
    print("targetPositions1:", targetPositions1)
    targetOrientation1 = p.getQuaternionFromEuler([3.14, 0, 0])
    print("targetOrientation1:", targetOrientation1)
    targetJointPositions1 = p.calculateInverseKinematics(robot1ID, endEffectorIndex, targetPositions1, targetOrientation1)
    print("targetJointPositions:", targetJointPositions1)
    p.setJointMotorControlArray(robot1ID, range(numJoints), p.POSITION_CONTROL, targetJointPositions1)

    # robot2
    targetPositions2 = [0, 2, 2]
    print("targetPositions2:", targetPositions2)
    targetOrientation2 = p.getQuaternionFromEuler([-3.14, 0, 0])
    print("targetOrientation2:", targetOrientation2)
    targetJointPositions2 = p.calculateInverseKinematics(robot2ID, endEffectorIndex, targetPositions2,
                                                         targetOrientation2)
    print("targetJointPositions2:", targetJointPositions2)
    p.setJointMotorControlArray(robot2ID, range(numJoints), p.POSITION_CONTROL, targetJointPositions2)

    slidingPosition = 6
    slidingVelocity = 10
    p.setJointMotorControl2(bodyUniqueId=robot1ID, jointIndex=slidingJointIndex,
                            controlMode=p.VELOCITY_CONTROL, targetVelocity=slidingVelocity, force=500)
    p.setJointMotorControl2(bodyUniqueId=robot2ID, jointIndex=slidingJointIndex,
                            controlMode=p.VELOCITY_CONTROL, targetVelocity=slidingVelocity, force=500)

    video = 'demo.mp4'
    frame_rate = 30
    videoWriter = imageio.get_writer(video, fps=frame_rate)

    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./10.)
        print("i:", i)
        currentPostion = p.getJointState(robot1ID, slidingJointIndex)[0]
        print("currentPostion:", currentPostion)

        # get camera image
        width, height, rgb_pixels, _, _ = p.getCameraImage(1920, 1080, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb_array = np.array(rgb_pixels, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (height, width, 4))
        rgb_array = rgb_array[:, :, :3]
        videoWriter.append_data(rgb_array)

        if abs(currentPostion - slidingPosition) < 0.01:
            p.setJointMotorControl2(
                bodyUniqueId=robot1ID,
                jointIndex=slidingJointIndex,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,  # 将目标速度设置为零，停止控制
                force=0
            )
            p.setJointMotorControl2(
                bodyUniqueId=robot2ID,
                jointIndex=slidingJointIndex,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,  # 将目标速度设置为零，停止控制
                force=0
            )
            break

    videoWriter.close()
    # close server
    p.disconnect()