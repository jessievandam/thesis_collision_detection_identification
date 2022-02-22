First of all, it should be noted that for the code to run, the rosbags are required with the recorded collision data. In the folder **function_files**, all methods described in the thesis are found as function files. Moreover, files to read data from the rosbags, and files to plot certain estimations are there. The main files, as explained below, contain descriptions on how to run them. 

## Step 0: calibration
Since the ground truth F/T sensor collision force data is recorded in a different rosbag than the rest of the data, `calibration.m` is run. With this, the data sets are manually synchronized. The obtained parameters from this are found in the folder **params_recordings**.

## Step 1: torque and force estimation
To compare the four observers as in the review paper, or to compare all state-of-the-art methods as in the method paper, the file `torque_estimation.m` is ran. It read the data from the rosbags, estimates the torques, and computes the delay and error.

## Step 2: collision detection
For this step, 2 files are present: `detection_filtering.m`, which filters the estimated forces in both time- and frequency-domain, and `detection_thresholds_isolation.m`. The latter compares the constant threshold to the dynamic threshold based on velocity and the dynamic threshold based on standard deviation. 

## Step 3: collision isolation
To check if collisions can be isolated on base/arm and on forearm/upperarm, the file `detection_thresholds_isolation.m` additionally contains a part that filters the estimated torques and sets a threshold for isolation.