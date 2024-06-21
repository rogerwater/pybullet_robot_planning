import time
import pybullet
import pybullet_data


if __name__ == '__main__':
    # connect to the PyBullet server
    client = pybullet.connect(pybullet.GUI)

    # config GUI
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    # add the resource path
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    # set gravity
    pybullet.setGravity(0, 0, -9.8)

    # load scene
    planeID = pybullet.loadURDF("plane.urdf")

    # load robot
    robotID = pybullet.loadURDF("r2d2.urdf", [0, 0, 1], [0, 0, 0, 1])

    # start rendering
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    # set real-time simulation
    pybullet.setRealTimeSimulation(1)
    while True:
        pass

    # close server
    pybullet.disconnect()
