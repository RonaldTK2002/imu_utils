#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def magnetic_callback(msg):
    """Convert magnetic field data to an RViz Marker."""
    marker = Marker()
    marker.header.frame_id = "map"  # Change to match your frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "magnetic_field"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    # Define the start and end points of the arrow
    start = Point(0, 0, 0)
    end = Point(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    marker.points.append(start)
    marker.points.append(end)

    # Set arrow color and size
    marker.scale.x = 0.02  # Arrow shaft diameter
    marker.scale.y = 0.05  # Arrow head size
    marker.scale.z = 0.05
    marker.color.a = 1.0  # Opacity
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0  # Red arrow

    marker_pub.publish(marker)

rospy.init_node("magnetic_field_visualizer")
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
rospy.Subscriber("/magnetometer_topic", MagneticField, magnetic_callback)

rospy.spin()

