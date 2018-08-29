import numpy as np

# Load the GPS X signal
gps_data = np.loadtxt('config/log/Graph1.txt', delimiter=',', dtype='Float64', skiprows=1)

# Load the IMU Accelerometer X signal
acc_data = np.loadtxt('config/log/Graph2.txt', delimiter=',', dtype='Float64', skiprows=1)

# Calculate the standard deviation and print result
print("Standard deviation of GPS: {}".format(np.std(gps_data[:,1])))
print("Standard deviation of accelerometer: {}".format(np.std(acc_data[:,1])))