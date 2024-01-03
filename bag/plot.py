import rosbag
import matplotlib.pyplot as plt

def extract_data(bag, topics):
    timestamps = []
    RMSE_p = []
    RMSE_v = []

    for topic, msg, t in bag.read_messages(topics):
    # Extract relevant data from the message
        timestamps.append(msg.header.stamp.to_sec())
        RMSE_p.append(msg.RMSE_p)
        RMSE_v.append(msg.RMSE_v)

    return timestamps, RMSE_p, RMSE_v

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

def plotFromBag(bag, name):
    
    timestamps, EIF_1_RMSE_p, EIF_1_RMSE_v = extract_data(bag, '/iris_1/SHEIF/RMSE')
    bag.close()
    plot_data_p(timestamps, EIF_1_RMSE_p, name)
    plot_data_v(timestamps, EIF_1_RMSE_v, name)

folder = '/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/'

file = folder + 'SHEIF_3hz.bag'
bag = rosbag.Bag(file)
plotFromBag(bag, 'SHEIF pose rate 3hz')