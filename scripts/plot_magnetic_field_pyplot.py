#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import MagneticField

# Data storage
time_vals = []
mag_x = []
mag_y = []
mag_z = []

def magnetic_callback(msg):
    current_time = rospy.Time.now().to_sec()
    time_vals.append(current_time)
    mag_x.append(msg.magnetic_field.x)
    mag_y.append(msg.magnetic_field.y)
    mag_z.append(msg.magnetic_field.z)

    # Keep only the last 100 points for real-time plotting
    if len(time_vals) > 100:
        time_vals.pop(0)
        mag_x.pop(0)
        mag_y.pop(0)
        mag_z.pop(0)

    # Plot the data
    plt.clf()
    plt.plot(time_vals, mag_x, label="X")
    plt.plot(time_vals, mag_y, label="Y")
    plt.plot(time_vals, mag_z, label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Magnetic Field (Tesla)")
    plt.legend()
    plt.pause(0.05)

if __name__ == "__main__":
    rospy.init_node("magnetic_plotter")
    rospy.Subscriber("/magnetometer_topic", MagneticField, magnetic_callback)
    plt.ion()  # Interactive mode
    plt.show()
    rospy.spin()

