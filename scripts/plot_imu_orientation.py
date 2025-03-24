import rospy
import matplotlib.pyplot as plt
from bagpy import bagreader
import pandas as pd
import numpy as np
import sys
import os
from tf.transformations import euler_from_quaternion
import gmplot

# Lists to store extracted data
time_imu_vals = []
orientation_yaw = []
position_x = []
position_y = []
coordinates = []


def extract_imu(bagfile):
    # Read IMU data
    imu_csv_path = bagfile.message_by_topic('/microstrain/imu/data')
    imu_data = pd.read_csv(imu_csv_path)

    # Extract quaternion components
    qx_list = imu_data['orientation.x'].values
    qy_list = imu_data['orientation.y'].values
    qz_list = imu_data['orientation.z'].values
    qw_list = imu_data['orientation.w'].values
    time_list = imu_data['Time'].values  # Extract time

    for i in range(len(qx_list)):
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion([qx_list[i], qy_list[i], qz_list[i], qw_list[i]])

        time_imu_vals.append(time_list[i])  # Corrected variable
        orientation_yaw.append(np.degrees(yaw))  # Convert yaw to degrees


def extract_odom(bagfile):
    # Read odometry data
    odom_csv_path = bagfile.message_by_topic('/lego_loam/odom')
    odom_data = pd.read_csv(odom_csv_path)

    global position_x, position_y
    position_x = odom_data['pose.pose.position.x'].values
    position_y = odom_data['pose.pose.position.y'].values
    
def extract_gps (bagfile):
    
    gps_csv_path = bagfile.message_by_topic('/reach/fix')
    gps_data = pd.read_csv(gps_csv_path)
    
    for i in range(len(gps_data['latitude'].values)):
        gps = (gps_data['latitude'].values[i],gps_data['longitude'].values[i])
        coordinates.append(gps)
    rospy.loginfo(coordinates)
        
    
def plot_map():
    global coordinates
    apikey = 'AIzaSyABXV1ZF2nogIv0VMGpOSlAK7tebLcM_40'
    gmap = gmplot.GoogleMapPlotter(coordinates[0][0], coordinates[0][1], 14, apikey=apikey)    
    trajectory = zip(*coordinates)
    gmap.polygon(*trajectory, color='cornflowerblue', edge_width=10)
    gmap.draw("../graphs/map.html")
    
def plot():
    fig, axes = plt.subplots(2, 1, figsize=(8, 6))  # 2 rows, 1 column

    # Plot 1 - IMU Yaw
    axes[0].plot(time_imu_vals, orientation_yaw, label="Yaw", color="b")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Yaw (degrees)")
    axes[0].set_title("IMU Orientation - Yaw")
    axes[0].legend()
    axes[0].grid()

    # Plot 2 - Odometry
    axes[1].plot(position_x, position_y, label="Odometry", color="r")
    axes[1].set_xlabel("x (m)")
    axes[1].set_ylabel("y (m)")
    axes[1].set_title("Position - Odometry")
    axes[1].legend()
    axes[1].grid()

    # Adjust layout
    plt.tight_layout()
    # Save the figure
    file_name = os.path.splitext(sys.argv[1])[0]
    plt.savefig(f"../graphs/{file_name}.png")
    
    plot_map()

    
    # Show the figure (optional)
    plt.show()


if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            print("Please provide the bag file path")
            sys.exit(0)

        # Construct the full path to the .bag file
        bagfile_dir = "../bag_files/" + sys.argv[1]

        # Initialize ROS node
        rospy.init_node("plot_imu_orientation")

        bag = bagreader(bagfile_dir)

        # Extract and plot data
        extract_odom(bag)
        extract_imu(bag)
        extract_gps(bag)
        plot()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted. Shutting down.")
        sys.exit(0)
