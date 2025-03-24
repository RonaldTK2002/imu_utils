#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import MagneticField

# Threshold for abnormal magnetic interference (adjust as needed)
EARTH_MAG_FIELD_UT = 50  # Expected Earth's field in µT (approximate)
INTERFERENCE_THRESHOLD = 100  # Above this, likely interference

def magnetic_callback(msg):
    # Convert Tesla to microtesla (µT)
    bx_ut = msg.magnetic_field.x * 1e6
    by_ut = msg.magnetic_field.y * 1e6
    bz_ut = msg.magnetic_field.z * 1e6

    # Compute total magnetic field strength
    b_total_ut = math.sqrt(bx_ut**2 + by_ut**2 + bz_ut**2)

    # Detect interference
    if abs(b_total_ut - EARTH_MAG_FIELD_UT) > INTERFERENCE_THRESHOLD:
        rospy.logwarn(f"Magnetic interference detected! Field: {b_total_ut:.2f} µT")
    else:
        rospy.loginfo(f"Magnetic field: {b_total_ut:.2f} µT (Normal)")

def main():
    rospy.init_node("magnetic_interference_detector")
    rospy.Subscriber("/magnetometer_topic", MagneticField, magnetic_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

