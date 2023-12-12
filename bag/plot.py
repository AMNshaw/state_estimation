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
    plt.plot(timeStamps, RMSE_p, label='position RMSE (m)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position root mean square error (m)')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.annotate(f'Average RMSE: {average_RMSE_p:.2f}', xy=(timeStamps[0], max(RMSE_p)), color='red')
    plt.show()

def plot_data_v(timeStamps, RMSE_v, title):
    average_RMSE_v = sum(RMSE_v)/len(RMSE_v)

    plt.figure(figsize=(10, 6))
    plt.plot(timeStamps, RMSE_v, label='velocity RMSE (m/s)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity root mean square error (m/s)')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.annotate(f'Average RMSE: {average_RMSE_v:.2f}', xy=(timeStamps[0], max(RMSE_v)), color='red')
    plt.show()

def plotFromBag(bag, name):
    if(name == 'HEIF'):
        timestamps1, HEIF_RMSE_p, HEIF_RMSE_v = extract_data(bag, '/HEIF/RMSE')
        timestamps2, EIF_1_RMSE_p, EIF_1_RMSE_v = extract_data(bag, '/iris_1/TEIF/RMSE')
        timestamps3, EIF_2_RMSE_p, EIF_2_RMSE_v = extract_data(bag, '/iris_2/TEIF/RMSE')
        bag.close()
        plot_data_p(timestamps1, HEIF_RMSE_p, 'HEIF')
        plot_data_v(timestamps1, HEIF_RMSE_v, 'HEIF')
        plot_data_p(timestamps2, EIF_1_RMSE_p, 'EIF1')
        plot_data_v(timestamps2, EIF_1_RMSE_v, 'EIF1')
        plot_data_p(timestamps3, EIF_2_RMSE_p, 'EIF2')
        plot_data_v(timestamps3, EIF_2_RMSE_v, 'EIF2')
    
    if(name=='EIF1'):
        timestamps, EIF_1_RMSE_p, EIF_1_RMSE_v = extract_data(bag, '/iris_1/TEIF/RMSE')
        bag.close()
        plot_data_p(timestamps, EIF_1_RMSE_p, 'EIF1')
        plot_data_v(timestamps, EIF_1_RMSE_v, 'EIF1')
    
    if(name=='EIF2'):
        timestamps, EIF_2_RMSE_p, EIF_2_RMSE_v = extract_data(bag, '/iris_2/TEIF/RMSE')
        bag.close()
        plot_data_p(timestamps, EIF_2_RMSE_p, 'EIF2')
        plot_data_v(timestamps, EIF_2_RMSE_v, 'EIF2')

bag0 = rosbag.Bag('/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/HEIF.bag')
bag1 = rosbag.Bag('/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/EIF_1.bag')
bag2 = rosbag.Bag('/home/ncrl/gazebo_sim_ws/src/state_estimation/bag/EIF_2.bag')


plotFromBag(bag0, 'HEIF')
