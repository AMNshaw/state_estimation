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

def plot_position_3D(GT_poses, est_poses):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    GT_x = [pose.position.x for pose in GT_poses]
    GT_y = [pose.position.y for pose in GT_poses]
    GT_z = [pose.position.z for pose in GT_poses]
    
    est_x = [pose.position.x for pose in est_poses]
    est_y = [pose.position.y for pose in est_poses]
    est_z = [pose.position.z for pose in est_poses]

    ax.plot(GT_x, GT_y, GT_z, label='GT Pose')
    ax.plot(est_x, est_y, est_z, label='Est Pose')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()
    
    plt.title('3D Poses')
    plt.show()

def plot_data_p(timeStamps, RMSE_p, title):
    average_RMSE_p = sum(RMSE_p)/len(RMSE_p)

    plt.figure(figsize=(10, 6))
    plt.plot(timeStamps, RMSE_p, label='position')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position root mean square error (m)')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.annotate(f'Average RMSE: {average_RMSE_p:.3f}', xy=(timeStamps[0], max(RMSE_p)), color='red')
    plt.show()

def plot_data_v(timeStamps, RMSE_v, title):
    average_RMSE_v = sum(RMSE_v)/len(RMSE_v)

    plt.figure(figsize=(10, 6))
    plt.plot(timeStamps, RMSE_v, label='velocity')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity root mean square error (m/s)')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.annotate(f'Average RMSE: {average_RMSE_v:.3f}', xy=(timeStamps[0], max(RMSE_v)), color='red')
    plt.show()

def plot_position_error(timeStamps, E_p, axis):
    average_error = sum(E_p)/len(E_p)

    plt.figure(figsize=(10, 6))
    plt.plot(timeStamps, E_p, label='Error')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Error_'+ axis)
    plt.title(axis+'_error ')
    plt.legend()
    plt.grid(True)
    plt.annotate(f'Average Error: {average_error:.3f}', xy=(timeStamps[0], max(E_p)), color='red')
    plt.show()

def plotFromBag(bag, name):
    E_x = []
    E_y = []
    E_z = []
    timestamps, EIF_1_GTpose, EIF_1_Estpose, EIF_1_RMSE_p, EIF_1_RMSE_v = extract_data(bag, '/iris_1/SHEIF/Plot')
    # timestamps, EIF_2_GTpose, EIF_2_Estpose, EIF_2_RMSE_p, EIF_2_RMSE_v = extract_data(bag, '/iris_2/SHEIF/Plot')
    # timestamps, EIF_3_GTpose, EIF_3_Estpose, EIF_3_RMSE_p, EIF_3_RMSE_v = extract_data(bag, '/iris_3/SHEIF/Plot')
    for (GTpose, Estpose) in zip(EIF_1_GTpose, EIF_1_Estpose):
        E_x.append(abs(GTpose.position.x - Estpose.position.x))
        E_y.append(abs(GTpose.position.y - Estpose.position.y))
        E_z.append(abs(GTpose.position.z - Estpose.position.z))

    bag.close()
    
    plot_position_3D(EIF_1_GTpose, EIF_1_Estpose)
    plot_position_error(timestamps, E_x, 'x')
    plot_position_error(timestamps, E_y, 'y')
    plot_position_error(timestamps, E_z, 'z')
    plot_data_p(timestamps, EIF_1_RMSE_p, name)
    plot_data_v(timestamps, EIF_1_RMSE_v, name)

folder = '/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/'

file = folder + 'all_3hz.bag'
print(file)
bag = rosbag.Bag(file)
plotFromBag(bag, 'SHEIF, all robots has absolute position rate 3hz')