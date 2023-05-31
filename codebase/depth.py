#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage

# Callback function that gets called each time a message is received.
def callback(data):
    # For now, just print the data.
    rospy.loginfo("Received image data: %s", data)

def listener():
    # Initiate this script as a ROS node named 'image_listener'.
    rospy.init_node('image_listener', anonymous=True)

    # Subscribe to the /camera/color/image_raw/compressed topic with CompressedImage as its message type.
    rospy.Subscriber("/camera/depth/image_rect_raw", CompressedImage, callback)

    # Keep the node running until it's shut down.
    rospy.spin()

if __name__ == '__main__':
    print('DEPTH_LISTENER_RUNNING')
    listener()
