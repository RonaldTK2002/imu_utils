#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import MagneticField
from math import Pi

def magnetic_callback(msg):
    """Converts magnetic field data to a TF2 transform"""
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"  # Change this to your desired frame
    t.child_frame_id = "magnetic_field"

    # Normalize the vector (for visualization only)
    mag_x = msg.magnetic_field.x*2
    mag_y = msg.magnetic_field.y
    mag_z = msg.magnetic_field.z
    norm = (mag_x**2 + mag_y**2 + mag_z**2) ** 0.5

    if norm > 0:
        mag_x /= norm
        mag_y /= norm
        mag_z /= norm

    # Convert the magnetic field vector into a rotation (approximation)
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    # Set orientation using a quaternion (assuming a simple alignment)
    t.transform.rotation.x = mag_x
    t.transform.rotation.y = mag_y
    t.transform.rotation.z = mag_z
    t.transform.rotation.w = 1.0  # Normalize for valid quaternion

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("magnetic_tf_broadcaster")
    rospy.Subscriber("/magnetometer_topic", MagneticField, magnetic_callback)
    rospy.spin()

