#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

# Global variables
color_image = None
depth_image = None
fx = 0.0  # Focal length x
fy = 0.0  # Focal length y
cx = 0.0  # Optical center x
cy = 0.0  # Optical center y
bridge = CvBridge()


def color_callback(msg):
    global color_image
    color_image = bridge.imgmsg_to_cv2(msg, "bgr8")


def depth_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg)


def camera_info_callback(data):
    global fx, fy, cx, cy
    fx = data.P[0]
    fy = data.P[5]
    cx = data.P[2]
    cy = data.P[6]


def block_detector():
    rospy.init_node('block_detector', anonymous=True)
    
    rospy.Subscriber('/camera/color/image_raw', Image, color_callback)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    pub = rospy.Publisher('/block_mask', Image, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if color_image is not None and depth_image is not None:
            # convert to grayscale
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            mask = np.zeros_like(gray)

            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
                if len(approx) == 4:  # we assume blocks are rectangular
                    cv2.drawContours(mask, [approx], 0, 255, -1)  # fill the contour

                    # find the center of the block
                    M = cv2.moments(approx)
                    if M["m00"] != 0:  # to avoid division by zero
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0

                    Z = depth_image[cY, cX] / 1000.0  # Assuming depth is in millimeters
                    X = (cX - cx) * Z / fx
                    Y = (cY - cy) * Z / fy

                    print("3D position of block: X={}, Y={}, Z={}".format(X, Y, Z))

            mask_msg = bridge.cv2_to_imgmsg(mask, "mono8")
            pub.publish(mask_msg)

        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        block_detector()
    except rospy.ROSInterruptException:
        pass
