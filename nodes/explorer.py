#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap, GetMapResponse

from move_base_client import GoalSender

# Random Explorer Class
class RandomExplorer:

    def __init__(self):
        # Define a Service for Map
        rospy.wait_for_service('/dynamic_map')
        self.get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)

        # Define a Simple Goal Client
        self.goal_sender = GoalSender()
        
        self.latest_map_msg = None
        self.latest_map = None

    def update_map(self):

        try:
            map_resp = self.get_map_srv()
            assert isinstance(map_resp, GetMapResponse)

            if map_resp.map.info.width <= 64:  #Default size of map before any input (64x64)
                return False

            self.latest_map_msg = map_resp.map
            self.latest_map = np.array(self.latest_map_msg.data)
            print(self.latest_map_msg.info)

        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed to get map: %s" % e)
            return False

        return True

    def process_map(self):

        # Get Map Metadata
        width = self.latest_map_msg.info.width
        height = self.latest_map_msg.info.height
        res = self.latest_map_msg.info.resolution
        map_origin = np.array([self.latest_map_msg.info.origin.position.x, self.latest_map_msg.info.origin.position.y])
        map_frame_id = self.latest_map_msg.header.frame_id
        stamp = self.latest_map_msg.header.stamp

        # Pick Goal and Create Msg
        cells_to_pick = self.get_valid_cells(height, self.latest_map, width)
        goal = self.get_goal(cells_to_pick, map_origin, res)
        goal_msg = self.make_goal_msg(goal, map_frame_id)

        return goal_msg

    # Return explored cells
    def get_valid_cells(self, height, gridmap, width):
        # Get Number of Explored Cells
        cells_explored = np.count_nonzero(gridmap > -1)
        rospy.loginfo("Cells Explored %i", cells_explored)

        # Create an Array with Cells to Pick
        cells = 0
        cells_to_pick = np.zeros((cells_explored, 2))
        for y in xrange(0, height):
            for x in xrange(0, width):
                idx = x + y * width
                if gridmap[idx] > -1:
                    cells_to_pick[cells][0] = x
                    cells_to_pick[cells][1] = y
                    cells = cells + 1
        return cells_to_pick

    # Select Random Valid Cell
    def get_goal(self, cells_to_pick, map_origin, res):
        # Pick Cell
        rand_idx = np.random.randint(0, len(cells_to_pick))

        goal = cells_to_pick[rand_idx]
        goal = (goal * res) + map_origin
        return goal

    def make_goal_msg(self, goal, frame_id="map"):
        # Get Message
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 0
        goal_msg.pose.orientation.w = 1
        goal_msg.header.frame_id = frame_id
        goal_msg.header.stamp = rospy.Time.now()
        return goal_msg

    def send_goal(self, goal):
        raise NotImplementedError()

    def loop(self):

        while not rospy.is_shutdown():
            rospy.loginfo("New Goal...")

            if self.update_map():
                goal_msg = self.process_map()
                self.goal_sender.send_goal(goal_msg)


if __name__ == '__main__':
    rospy.init_node('random_explorer')
    explorer = RandomExplorer()
    explorer.loop()
