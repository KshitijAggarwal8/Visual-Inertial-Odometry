import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import numpy as np

# File path to the text file
file_path = 'groundtruth.txt'  # Replace with the path to your file

# IMU data is stored in text file with the following format:
# id timestamp ang_vel_x ang_vel_y ang_vel_z lin_acc_x lin_acc_y lin_acc_z
file_path_imu = 'imu.txt'  

# Lists to store the tx, ty, tz values
tx, ty, tz = [], [], []
qx, qy, qz, qw = [], [], [], []
gt_roll, gt_pitch, gt_yaw = [], [], []

set_R = False
initial_R = False

t_min = 0
t_max = 0

# Reading the file
with open(file_path, 'r') as file:
    for line in file:
        # Skip lines that start with '#', as they are comments
        if line.startswith('#'):
            continue
        
        # Split the line by spaces
        values = line.split()
        
        # Extract tx, ty, tz (index 1, 2, 3 correspond to tx, ty, tz)
        tx.append(float(values[1])), ty.append(float(values[2])), tz.append(float(values[3]))
        # Extract qx, qy, qz, qw (index 4, 5, 6, 7 correspond to qx, qy, qz, qw)
        qx.append(float(values[4])), qy.append(float(values[5])), qz.append(float(values[6])), qw.append(float(values[7]))

        if set_R == False:
            set_R = True
            initial_R = R.from_quat([float(values[4]), float(values[5]), float(values[6]), float(values[7])]).as_matrix()

        # Unrotate the orientation values so that the initial orientation is the identity matrix
        r = R.from_quat([float(values[4]), float(values[5]), float(values[6]), float(values[7])]).as_matrix()
        r = initial_R.T @ r
        r_ = R.from_matrix(r).as_euler('xyz', degrees=True)
        gt_roll.append(r_[0]), gt_pitch.append(r_[1]), gt_yaw.append(r_[2])

        # Convert r to quaternion
        q_ = R.from_matrix(r).as_quat()
        qx[-1] = q_[0]
        qy[-1] = q_[1]
        qz[-1] = q_[2]
        qw[-1] = q_[3]

        # Automate the process of finding the min and max time values
        if t_min == 0:
            t_min = float(values[0])
        t_max = float(values[0])

# Convert lists to numpy arrays
t_x = np.array(tx)
t_y = np.array(ty)
t_z = np.array(tz)

# Subtract the initial position from the position values
t_x = t_x - t_x[0]
t_y = t_y - t_y[0]
t_z = t_z - t_z[0]

# Stack the position values into a single array of 3xn
gt = np.vstack((t_x, t_y, t_z))

# Get initial orientation
r = R.from_quat([qx[0], qy[0], qz[0], qw[0]]).as_matrix()

# Unrotate the position values
gt = r.T @ gt

# Pull t_x, t_y, t_z from gt
t_x = gt[0, :]
t_z = -gt[1, :]
t_y = gt[2, :]

# Convert back to lists
tx = t_x.tolist()
ty = t_y.tolist()
tz = t_z.tolist()

####################################################################################################

x, y, z = [], [], []
roll, pitch, yaw = [], [], []

with open("output_vo.txt", "r") as file:
    for line in file:
        if line.startswith("x:"):
            x.append(float(line.split()[1]))
        elif line.startswith("y:"):
            y.append(float(line.split()[1]))
        elif line.startswith("z:"):
            z.append(float(line.split()[1]))
        elif line.startswith("roll:"):
            roll.append(float(line.split()[1]))
        elif line.startswith("pitch:"):
            pitch.append(float(line.split()[1]))
        elif line.startswith("yaw:"):
            yaw.append(float(line.split()[1]))

time_gt = np.linspace(0, 30, len(qx))
time_vo = np.linspace(0, 30, len(roll))

# Create a 2D plot of output_roll, gt_roll, output_pitch, gt_pitch, output_yaw, gt_yaw in a subplot
fig, axs = plt.subplots(3, 1, figsize=(10, 12))

# Plot output_roll, gt_roll
axs[0].plot(time_vo, roll, label='vo roll')
axs[0].plot(time_gt, gt_roll, label='gt roll')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Roll (degrees)')
axs[0].set_title('Roll Comparison')
axs[0].legend()
axs[0].grid()

# Plot output_pitch, gt_pitch
axs[1].plot(time_vo, pitch, label='vo pitch')
axs[1].plot(time_gt, gt_pitch, label='gt pitch')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Pitch (degrees)')
axs[1].set_title('Pitch Comparison')
axs[1].legend()
axs[1].grid()

# Plot output_yaw, gt_yaw
axs[2].plot(time_vo, yaw, label='vo yaw')
axs[2].plot(time_gt, gt_yaw, label='gt yaw')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Yaw (degrees)')
axs[2].set_title('Yaw Comparison')
axs[2].legend()
axs[2].grid()

plt.tight_layout()
plt.show()