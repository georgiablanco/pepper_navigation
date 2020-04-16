#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from __future__ import print_function
import qi
import argparse
import sys
import time
import almath
import math
import rospy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
import dlib
import cv2
import roslib
from sensor_msgs.msg import CompressedImage
from scipy.ndimage import filters
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import sensor_msgs.point_cloud2
from naoqi import ALProxy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

# prefix to the names of dummy fields we add to get byte alignment correct. this needs to not
# clash with any actual field names
DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


class PepperTracker(object):

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(PepperTracker, self).__init__()
        app.start()
        session = app.session
        self.motion = session.service("ALMotion")
        self.navigation = session.service("ALNavigation")
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.parseBoundingBoxes, queue_size=1)
        self.names = ["HeadYaw", "HeadPitch"]
        self.angles = [0, 0]
        self.fractionMaxSpeed = 0.05
        self.started_tracking = False
        self.subscriber = rospy.Subscriber("/pepper_robot/camera/front/image_rect_color/compressed", CompressedImage,
                                           self.image_tracking_callback, queue_size=1)
        self.bounding_box = BoundingBox()
        self.person_seen = False
        self.head_offcentre = False
        # Subscribe to point cloud
        self.point_cloud_subscriber = rospy.Subscriber(
            "/pepper_local_republisher/pepper_robot/camera/depth_registered/transformed_points", PointCloud2,
            self.savePointCloud, queue_size=1)
        # roslaunch pepper_local_republisher point_cloud_transformer_node.launch

        self.tracker = dlib.correlation_tracker()
        self.win = dlib.image_window()

        # Configure these based on the image size
        self.image_height = 320
        self.image_width = 240
        self.image_center_x = self.image_width / 2
        self.image_center_y = self.image_height / 2
        # Tune these appropriately
        # Thresholds at which the target is determined to be in the center of the image
        self.x_deadspot = 320  # 60 --> 40 --> 35 --> 90 --> 150 --> 160 --> 320
        self.y_deadspot = 420  # 80 --> 120 --> 160 --> 140 --> 180 --> 240 --> 300 --> 330 --> 420
        # Scale factor that determines the speed at which Pepper will turn its head. Large numbers will slow down the speed while
        # lower numbers will increase the speed
        self.x_movement_scale = 10.0  # 2.0 --> 3.0 --> 5.0 -->  7.0 --> 10.0
        self.y_movement_scale = 10.0  # 2.0 --> 3.0 --> 5.0 -->  7.0 --> 10.0
        # Threshold at which the head is turned too far and thus, the body will start turning instead
        self.head_x_threshold = 85.0 * almath.TO_RAD
        self.rotation_speed = 0.05  # 0.5 --> 0.25 --> 0.10

    def fields_to_dtype(self, fields, point_step):
        '''Convert a list of PointFields to a numpy record datatype.
        '''
        offset = 0
        np_dtype_list = []
        for f in fields:
            while offset < f.offset:
                # might be extra padding between fields
                np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1

            dtype = pftype_to_nptype[f.datatype]
            if f.count != 1:
                dtype = np.dtype((dtype, f.count))

            np_dtype_list.append((f.name, dtype))
            offset += pftype_sizes[f.datatype] * f.count

        # might be extra padding between points
        while offset < point_step:
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        return np_dtype_list

    def savePointCloud(self, cloud_msg, squeeze=True):
        # Turn the ROS Pointcloud 2 message into a numpy array and save it into self.point_cloud
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = self.fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
        # parse the cloud into an array
        self.cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
        # remove the dummy fields that were added
        self.cloud_arr = self.cloud_arr[
            [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
        if squeeze and cloud_msg.height == 1:
            self.cloud_arr = np.reshape(self.cloud_arr, (cloud_msg.width,))
        else:
            self.cloud_arr = np.reshape(self.cloud_arr, (cloud_msg.height, cloud_msg.width))

    def parseBoundingBoxes(self, msg):
        for bounding_box in msg.bounding_boxes:
            if bounding_box.Class == 'person':
                if bounding_box.probability >= 0.93:
                    self.person_seen = True
                    self.bounding_box = bounding_box

    def image_tracking_callback(self, ros_data):

        # image converted to numpy array
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # numpy array converted into dlib array
        # dlib.convert_image(image_np)

        if self.started_tracking is False and self.person_seen is True:
            self.tracker.start_track(image_np, dlib.rectangle(self.bounding_box.xmin, self.bounding_box.ymin,
                                                              self.bounding_box.xmax, self.bounding_box.ymax))
            self.started_tracking = True

        elif self.started_tracking is True and self.person_seen is True:
            # Else we just attempt to track from the previous frame
            self.tracker.update(image_np)
            self.win.clear_overlay()
            self.win.set_image(image_np)
            self.win.add_overlay(self.tracker.get_position())

            # Get the current location of the dlib bounding box
            current_tracking_bounding_box = self.tracker.get_position()

            # Dlib.point
            center_of_bounding_box = current_tracking_bounding_box.center()

            currentHeadYawAngle = self.angles[0]
            currentHeadPitchAngle = self.angles[1]

            if (center_of_bounding_box.x > self.image_center_x - self.x_deadspot) and (
                    center_of_bounding_box.x < self.image_center_x + self.x_deadspot):
                # target is in the center of the image
                # no need to do anything
                self.motion.move(0, 0, 0)
            elif (center_of_bounding_box.x < self.image_center_x - self.x_deadspot):
                # Target is in the left of the image, turn the head left
                self.angles[0] = self.angles[0] + ((
                                                               self.image_center_x - self.x_deadspot - center_of_bounding_box.x) / self.x_movement_scale) * almath.TO_RAD
            elif (center_of_bounding_box.x > self.image_center_x + self.x_deadspot):
                # Target is in the right of the image, turn the head right
                self.angles[0] = self.angles[0] - ((
                                                               center_of_bounding_box.x - self.image_center_x - self.x_deadspot) / self.x_movement_scale) * almath.TO_RAD

            if (self.angles[0] > self.head_x_threshold):
                # Check to see if the head is turned too far to the left
                # If so, turn the body to the left
                self.motion.move(0.0, 0.0, self.rotation_speed)
                self.angles[0] = currentHeadYawAngle
            elif (self.angles[0] < -self.head_x_threshold):
                # Check to see if the head is turned too far to the right
                # If so turn the body to the right
                self.motion.move(0.0, 0.0, -self.rotation_speed)
                self.angles[0] = currentHeadYawAngle
            else:
                self.motion.move(0, 0, 0)

            if (center_of_bounding_box.y > self.image_center_y - self.y_deadspot) and (
                    center_of_bounding_box.y < self.image_center_y + self.y_deadspot):
                # target is in the center of the image
                # no need to do anything
                pass
            elif (center_of_bounding_box.y < self.image_center_y - self.y_deadspot):
                # Target is above the center of the image, move the head up and make sure that the angle is within the joint limits
                self.angles[1] = max(self.angles[1] - ((
                                                                   self.image_center_y - self.y_deadspot - center_of_bounding_box.y) / self.y_movement_scale) * almath.TO_RAD,
                                     -40.0 * almath.TO_RAD)
            elif (center_of_bounding_box.y > self.image_center_y + self.y_deadspot):
                # Target is below the center of the image, move the head down and make sure that the angle is within the joint limits
                self.angles[1] = min(self.angles[1] + ((
                                                                   center_of_bounding_box.y - self.image_center_y - self.y_deadspot) / self.y_movement_scale) * almath.TO_RAD,
                                     25.0 * almath.TO_RAD)

            self.motion.setAngles(self.names, self.angles, self.fractionMaxSpeed)

            int_center_of_bounding_box_x = int(center_of_bounding_box.x)
            int_center_of_bounding_box_y = int(center_of_bounding_box.y)
            num_non_nans = 0
            total_x = 0
            total_y = 0
            for i in range(-1, 2):
                if (i + int_center_of_bounding_box_x) > 0 and (i + int_center_of_bounding_box_x) < 640:
                    for j in range(-1, 2):
                        if (j + int_center_of_bounding_box_y) > 0 and (j + int_center_of_bounding_box_y) < 320:
                            point = self.cloud_arr[int_center_of_bounding_box_x, int_center_of_bounding_box_y]
                            if (not math.isnan(point[0])):
                                total_x += point[0]
                                total_y += point[1]
                                num_non_nans += 1

            if not (num_non_nans is 0):
                # The target must be closer than 3m from the robot, otherwise the command will be ignored and a warning will be prompted
                targetX = float(total_x / num_non_nans)
                targetY = float(total_y / num_non_nans)
                self.navigation.navigateTo(targetX, targetY)
                # print("target x: ",targetX,"target Y: ",targetY)

            # x=self.cloud_arr.shape
            # print(x)
            # print(self.cloud_arr)
            # x=self.cloud_arr
            # print(x.shape)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="134.151.157.26",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["PepperTracker", "--qi-url=" + connection_url])

        try:
            rospy.init_node('pepper_tracker', anonymous=True)
            mad = PepperTracker(app)
            try:
                rospy.spin()
            except KeyboardInterrupt:
                print("Shutting down")

        except rospy.ROSInterruptException:
            print("Program interrupted before completion", file=sys.stderr)


    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
                                                                                             "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)