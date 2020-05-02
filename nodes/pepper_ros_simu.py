#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import pybullet
import pybullet_data
import time
from qibullet import PepperVirtual
from qibullet import PepperRosWrapper
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    # pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    urdf = pybullet.loadURDF("/home/georgiablanco/catkin_ws/src/pepper_navigation/worlds/restaurant.urdf")
    urdf = pybullet.loadURDF("/home/georgiablanco/catkin_ws/src/pepper_navigation/worlds/bullet_restaurant2.urdf")
    # urdf = pybullet.loadURDF("/home/georgiablanco/catkin_ws/src/pepper_navigation/worlds/samurai_tables.urdf")

    # sdf = pybullet.loadSDF("/home/georgiablanco/catkin_ws/src/pepper_navigation/worlds/samurai_tables1.sdf")

    wrap = PepperRosWrapper()
    # wrap.launchWrapper(pepper, "/naoqi_driver")
    wrap.launchWrapper(pepper, "/pepper")

    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)
    pepper.setAngles("HeadPitch", 0.1, 0.1)

    try:
        # time.sleep(1.0)
        # joint_parameters = list()
        #
        # for name, joint in pepper.joint_dict.items():
        #     if "Finger" not in name and "Thumb" not in name:
        #         joint_parameters.append((
        #             pybullet.addUserDebugParameter(
        #                 name,
        #                 joint.getLowerLimit(),
        #                 joint.getUpperLimit(),
        #                 pepper.getAnglesPosition(name)),
        #             name))
        #
        # while True:
        #     for joint_parameter in joint_parameters:
        #         pepper.setAngles(
        #             joint_parameter[1],
        #             pybullet.readUserDebugParameter(joint_parameter[0]), 1.0)

        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        wrap.stopWrapper()
        simulation_manager.stopSimulation(client)
