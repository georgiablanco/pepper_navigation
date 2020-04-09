#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import pybullet
import pybullet_data

from qibullet import PepperVirtual
from qibullet import PepperRosWrapper
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    # pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    urdf = pybullet.loadURDF(
        "/home/georgiablanco/catkin_ws/src/pepper_navigation/worlds/samurai_tables.urdf",
        globalScaling=0.5)


    wrap = PepperRosWrapper()
    wrap.launchWrapper(pepper, "/naoqi_driver")

    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        wrap.stopWrapper()
        simulation_manager.stopSimulation(client)
