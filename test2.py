import time
import pybullet
import pybullet_data


if __name__ == '__main__':
    client = pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    pybullet.setGravity(0, 0, -9.8)
    pybullet.setRealTimeSimulation(1)

    # 载入urdf格式是场景
    pybullet.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    # 载入urdf格式的机器人
    r_ind = pybullet.loadURDF('r2d2.urdf', (3, 1, 1), pybullet.getQuaternionFromEuler([0, 0, 0.785]))

    # 视觉属性
    visual_ind = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="C:\\Users\\ZHANGJin\\Desktop\\0000_asm.stp",
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=[0, 0, 0],
        meshScale=[1, 1, 1])

    # 从视觉和碰撞属性中创建模型
    pybullet.createMultiBody(
        baseMass=1,
        baseVisualShapeIndex=visual_ind,
        basePosition=[0, 0, 1],
        useMaximalCoordinates=True)

    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
    while True:
        time.sleep(1. / 240.)

