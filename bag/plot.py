import rosbag
import matplotlib.pyplot as plt

def extract_data(bag, topics):
    timestamps = []
    RMSE_p = []
    RMSE_v = []
    GT_poses = []
    est_poses = []

    for topic, msg, t in bag.read_messages(topics):
    # Extract relevant data from the message
        timestamps.append(msg.header.stamp.to_sec())
        RMSE_p.append(msg.RMSE_p)
        RMSE_v.append(msg.RMSE_v)
        GT_poses.append(msg.GT_pose)
        est_poses.append(msg.est_pose)

    return timestamps, GT_poses, est_poses, RMSE_p, RMSE_v

def plot_RMSE_p(timeStamps, RMSE_p, dataset_label):
    plt.figure(figsize=(10, 6))
    average_RMSE_p = sum(RMSE_p) / len(RMSE_p)  # Calculate average RMSE for position
    plt.plot(timeStamps, RMSE_p, label=f'Position RMSE for {dataset_label}')
    plt.axhline(y=average_RMSE_p, color='r', linestyle='--', label=f'Average RMSE: {average_RMSE_p:.3f}')  # Plot average RMSE line
    plt.text(timeStamps[int(len(timeStamps) / 10)], average_RMSE_p, f'Average RMSE: {average_RMSE_p:.3f}', color='red')  # Print average RMSE on plot
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position RMSE (m)')
    plt.title(f'Position RMSE for {dataset_label}')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_RMSE_v(timeStamps, RMSE_v, dataset_label):
    plt.figure(figsize=(10, 6))
    average_RMSE_v = sum(RMSE_v) / len(RMSE_v)  # Calculate average RMSE for velocity
    plt.plot(timeStamps, RMSE_v, label=f'Velocity RMSE for {dataset_label}', color='orange')
    plt.axhline(y=average_RMSE_v, color='r', linestyle='--', label=f'Average RMSE: {average_RMSE_v:.3f}')  # Plot average RMSE line
    plt.text(timeStamps[int(len(timeStamps) / 10)], average_RMSE_v, f'Average RMSE: {average_RMSE_v:.3f}', color='red')  # Print average RMSE on plot
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity RMSE (m/s)')
    plt.title(f'Velocity RMSE for {dataset_label}')
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_combined_position_3D(GT_poses1, est_poses1,
                               GT_poses2, est_poses2,
                                GT_poses3, est_poses3):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # First set of poses
    GT_x1 = [pose.position.x for pose in GT_poses1]
    GT_y1 = [pose.position.y for pose in GT_poses1]
    GT_z1 = [pose.position.z for pose in GT_poses1]
    est_x1 = [pose.position.x for pose in est_poses1]
    est_y1 = [pose.position.y for pose in est_poses1]
    est_z1 = [pose.position.z for pose in est_poses1]

    # Second set of poses
    GT_x2 = [pose.position.x for pose in GT_poses2]
    GT_y2 = [pose.position.y for pose in GT_poses2]
    GT_z2 = [pose.position.z for pose in GT_poses2]
    est_x2 = [pose.position.x for pose in est_poses2]
    est_y2 = [pose.position.y for pose in est_poses2]
    est_z2 = [pose.position.z for pose in est_poses2]

    # Third set of poses
    GT_x3 = [pose.position.x for pose in GT_poses3]
    GT_y3 = [pose.position.y for pose in GT_poses3]
    GT_z3 = [pose.position.z for pose in GT_poses3]
    est_x3 = [pose.position.x for pose in est_poses3]
    est_y3 = [pose.position.y for pose in est_poses3]
    est_z3 = [pose.position.z for pose in est_poses3]

    # Plotting
    ax.plot(GT_x1, GT_y1, GT_z1, label='GT Pose 1')
    ax.plot(est_x1, est_y1, est_z1, linestyle=':', label='Est Pose 1')
    ax.plot(GT_x2, GT_y2, GT_z2, label='GT Pose 2')
    ax.plot(est_x2, est_y2, est_z2, linestyle=':', label='Est Pose 2')
    ax.plot(GT_x3, GT_y3, GT_z3, label='GT Pose 3')
    ax.plot(est_x3, est_y3, est_z3, linestyle=':', label='Est Pose 3')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()

    plt.title('Combined 3D Poses')
    plt.show()

def plot_target_position_3D(GT_posest, est_posest):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Target set of poses
    GT_xt = [pose.position.x for pose in GT_posest]
    GT_yt = [pose.position.y for pose in GT_posest]
    GT_zt = [pose.position.z for pose in GT_posest]
    est_xt = [pose.position.x for pose in est_posest]
    est_yt = [pose.position.y for pose in est_posest]
    est_zt = [pose.position.z for pose in est_posest]

    # Plotting
    ax.plot(GT_xt, GT_yt, GT_zt, label='GT Pose target')
    ax.plot(est_xt, est_yt, est_zt, linestyle=':', label='Est Pose target')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()

    plt.title('Combined 3D Poses')
    plt.show()

def plotFromBag(bag, name):
    E_x = []
    E_y = []
    E_z = []
    # timestamps1, EIF_1_GTpose, EIF_1_Estpose, EIF_1_RMSE_p, EIF_1_RMSE_v = extract_data(bag, '/iris_1/SHEIF/Plot')
    # timestamps2, EIF_2_GTpose, EIF_2_Estpose, EIF_2_RMSE_p, EIF_2_RMSE_v = extract_data(bag, '/iris_2/SHEIF/Plot')
    # timestamps3, EIF_3_GTpose, EIF_3_Estpose, EIF_3_RMSE_p, EIF_3_RMSE_v = extract_data(bag, '/iris_3/SHEIF/Plot')
    timestampst, EIF_t_GTpose, EIF_t_Estpose, EIF_t_RMSE_p, EIF_t_RMSE_v = extract_data(bag, '/iris_1/THEIF/Plot')
    # for (GTpose, Estpose) in zip(EIF_1_GTpose, EIF_1_Estpose):
    #     E_x.append(abs(GTpose.position.x - Estpose.position.x))
    #     E_y.append(abs(GTpose.position.y - Estpose.position.y))
    #     E_z.append(abs(GTpose.position.z - Estpose.position.z))

    bag.close()
    
    # plot_combined_position_3D(EIF_1_GTpose, EIF_1_Estpose
    #                            , EIF_2_GTpose, EIF_2_Estpose
    #                            , EIF_3_GTpose, EIF_3_Estpose)
    
    plot_target_position_3D(EIF_t_GTpose, EIF_t_Estpose)

    # plot_RMSE_p(timestamps1, EIF_1_RMSE_p, "UAV1")
    # plot_RMSE_v(timestamps1, EIF_1_RMSE_v, "UAV1")
    # plot_RMSE_p(timestamps2, EIF_2_RMSE_p, "UAV2")
    # plot_RMSE_v(timestamps2, EIF_2_RMSE_v, "UAV2")
    # plot_RMSE_p(timestamps3, EIF_3_RMSE_p, "UAV3")
    # plot_RMSE_v(timestamps3, EIF_3_RMSE_v, "UAV3")
    plot_RMSE_p(timestampst, EIF_t_RMSE_p, "target")
    plot_RMSE_v(timestampst, EIF_t_RMSE_v, "target")
    #plot_combined_RMSE_v(timestamps1, EIF_1_RMSE_v, EIF_2_RMSE_v, EIF_3_RMSE_v)

def plot_combined_RMSE_p(RMSE_p1, label1, RMSE_p2, label2):
    plt.figure(figsize=(10, 6))
    
    # Normalize time steps: Start both from zero
    min_length = min(len(RMSE_p1), len(RMSE_p2))
    timeStamps1 = list(range(min_length))  # Creating a list from 0 to min_length
    timeStamps2 = list(range(min_length))

    # Cut the RMSE lists to match the new timestamps if necessary
    RMSE_p1 = RMSE_p1[:min_length]
    RMSE_p2 = RMSE_p2[:min_length]
    average_RMSE_p1 = sum(RMSE_p1) / len(RMSE_p1)
    average_RMSE_p2 = sum(RMSE_p2) / len(RMSE_p2)


    # Plot
    plt.plot(timeStamps1, RMSE_p1, label=f'Position RMSE for {label1}')
    plt.plot(timeStamps2, RMSE_p2, label=f'Position RMSE for {label2}')
    plt.axhline(y=average_RMSE_p1, color='b', linestyle='--', label=f'Average RMSE: {average_RMSE_p1:.3f} ({label1})')
    plt.axhline(y=average_RMSE_p2, color='r', linestyle='--', label=f'Average RMSE: {average_RMSE_p2:.3f} ({label2})')
    plt.xlabel('Normalized Time Steps')
    plt.ylabel('Position RMSE (m)')
    plt.title('Combined Position RMSE')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_combined_RMSE_v(RMSE_v1, label1, RMSE_v2, label2):
    plt.figure(figsize=(10, 6))
    
    # Normalize time steps: Start both from zero
    min_length = min(len(RMSE_v1), len(RMSE_v2))
    timeStamps1 = list(range(min_length))  # Creating a list from 0 to min_length
    timeStamps2 = list(range(min_length))

    # Cut the RMSE lists to match the new timestamps if necessary
    RMSE_v1 = RMSE_v1[:min_length]
    RMSE_v2 = RMSE_v2[:min_length]
    average_RMSE_v1 = sum(RMSE_v1) / len(RMSE_v1)
    average_RMSE_v2 = sum(RMSE_v2) / len(RMSE_v2)

    # Plot
    plt.plot(timeStamps1, RMSE_v1, label=f'Velocity RMSE for {label1}')
    plt.plot(timeStamps2, RMSE_v2, label=f'Velocity RMSE for {label2}')
    plt.axhline(y=average_RMSE_v1, color='b', linestyle='--', label=f'Average RMSE: {average_RMSE_v1:.3f} ({label1})')
    plt.axhline(y=average_RMSE_v2, color='r', linestyle='--', label=f'Average RMSE: {average_RMSE_v2:.3f} ({label2})')
    plt.xlabel('Normalized Time Steps')
    plt.ylabel('Velocity RMSE (m)')
    plt.title('Velocity RMSE')
    plt.legend()
    plt.grid(True)
    plt.show()


def plotFromTwoBags(file1, file2, topic, label1, label2):
    # Open both bag files
    bag1 = rosbag.Bag(file1)
    bag2 = rosbag.Bag(file2)

    # Extract data from both bags
    _, _, _, RMSE_p1, RMSE_v1 = extract_data(bag1, topic)
    _, _, _, RMSE_p2, RMSE_v2 = extract_data(bag2, topic)

    # Close the bag files
    bag1.close()
    bag2.close()

    # Plot combined RMSE for position from both bags with normalized time steps
    plot_combined_RMSE_p(RMSE_p1, label1, RMSE_p2, label2)
    plot_combined_RMSE_v(RMSE_v1, label1, RMSE_v2, label2)

folder = '/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/'

file1 = folder + 'QP.bag'
file2 = folder + 'noQP.bag'
bag1 = rosbag.Bag(file1)
topic = '/iris_1/THEIF/Plot'
plotFromTwoBags(file1, file2, topic, 'QP prediction', 'no QP prediction')
#plotFromBag(bag1, 'THEIF, Only one neigbor robots has absolute position rate 5hz')