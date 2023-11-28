import rosbag
import matplotlib.pyplot as plt

# Open the ROS Bag File
bag = rosbag.Bag('/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/EIF.bag')

# Extract Data
timestamps = []
values = []

for topic, msg, t in bag.read_messages(topics=['/iris_2/EIF/RMSE']):
    # Extract relevant data from the message
    timestamp = msg.header.stamp.to_sec()  # Convert ROS time to seconds
    value = msg.RMSE_p  # Modify this line to extract the specific data you need
    
    timestamps.append(timestamp)
    values.append(value)

average_value = sum(values) / len(values)


# Close the ROS Bag File
bag.close()

# Plot the Data
plt.figure(figsize=(10, 6))
plt.plot(timestamps, values, label='position RMSE (m)')
plt.xlabel('Time (seconds)')
plt.ylabel('Position root mean square error (m)')
plt.title('Single EIF')
plt.legend()
plt.grid(True)
plt.annotate(f'Average RMSE: {average_value:.2f}', xy=(timestamps[0], max(values)), color='red')

plt.show()



bag = rosbag.Bag('/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/HEIF.bag')

# Extract Data
timestamps = []
values = []

for topic, msg, t in bag.read_messages(topics=['/HEIF/RMSE']):
    # Extract relevant data from the message
    timestamp = msg.header.stamp.to_sec()  # Convert ROS time to seconds
    value = msg.RMSE_p  # Modify this line to extract the specific data you need
    
    timestamps.append(timestamp)
    values.append(value)

average_value = sum(values) / len(values)
# Close the ROS Bag File
bag.close()

# Plot the Data
plt.figure(figsize=(10, 6))
plt.plot(timestamps, values, label='position RMSE (m)')
plt.xlabel('Time (seconds)')
plt.ylabel('Position root mean square error (m)')
plt.title('HEIF')
plt.legend()
plt.grid(True)
plt.annotate(f'Average RMSE: {average_value:.2f}', xy=(timestamps[0], max(values)), color='red')

plt.show()
