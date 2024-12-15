#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import math
import pdb
# Sampling time
ref_sampleTime = .45
# ref_cell = [[np.linspace(0, total_simTime, int(total_simTime/ref_sampleTime)+1)*(5/3.6)],
#         [np.ones(int(total_simTime/ref_sampleTime)+1)*0.0],
#         [np.ones(int(total_simTime/ref_sampleTime)+1)*89*math.pi/180],
#         [np.ones(int(total_simTime/ref_sampleTime)+1)*(5/3.6)],
#         [np.ones(int(total_simTime/ref_sampleTime)+1)*(5/3.6)]]
speed = 3.5 # km/h
#rightmost ref
total_simTime1 = 8/speed*3.6
total_simTime2 = total_simTime1*2
x1 = np.ones(int(total_simTime1/ref_sampleTime)+1)*0.5
y1 = 8+np.linspace(0, total_simTime1, int(total_simTime1/ref_sampleTime)+1)*(-speed/3.6)
theta1 = -90*np.ones(int(total_simTime1/ref_sampleTime)+1)*math.pi/180
v1 = np.ones(int(total_simTime1/ref_sampleTime)+1)*speed/3.6

y2 = np.ones(int(total_simTime2/ref_sampleTime)+1)*-.5
x2 = 0.5+np.linspace(0, total_simTime2, int(total_simTime2/ref_sampleTime)+1)*(speed/3.6)
theta2 = -0*np.ones(int(total_simTime2/ref_sampleTime)+1)*math.pi/180
v2 = np.ones(int(total_simTime2/ref_sampleTime)+1)*speed/3.6

x = np.hstack((x1,x2))
y = np.hstack((y1,y2))
theta = np.hstack((theta1,theta2))
v = np.hstack((v1,v2))

ref_cell = [[x],[y],[theta],[v],[v]]
ref_defined = np.concatenate(ref_cell,axis = 0)
ref_defined[3:5,-1] = 0

# boundary_width = 6
y_values11 = np.linspace(9, -1.5, 106)
x_values11 = -1.4 * np.ones(106)
y_values21 = -1.5 * np.ones(195)
x_values21 = np.linspace(-1.4, 18, 195)
boundary11 = np.column_stack((x_values11, y_values11))
boundary21 = np.column_stack((x_values21, y_values21))

boundary1 = np.vstack((boundary11 , boundary21))


y_values12 = np.linspace(9, 1.5, 76)
x_values12 = 1.6 * np.ones(76)
y_values22 = 1 * np.ones(165)
x_values22 = np.linspace(1.6, 18, 165)

boundary12 = np.column_stack((x_values12, y_values12))

boundary22 = np.column_stack((x_values22, y_values22))

boundary2 = np.vstack((boundary12 ,boundary22))

step_Size = int(boundary1.shape[0]/56)
boundary1 = boundary1[0:-1:step_Size,0:]
boundary1 = boundary1[0:56,0:]



step_Size = int(boundary2.shape[0]/56)
boundary2 = boundary2[0:-1:step_Size,0:]
boundary2 = boundary2[0:56,0:]
center = np.column_stack((x,y))
lab_track = np.hstack((boundary1,center,boundary2))
np.save("exp_track.npy",lab_track)
# pdb.set_trace()
# Plot the reference trajectory and boundaries
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Reference Path', color='blue')
plt.scatter(x[0], y[0], color='green', label='Start Point')
plt.scatter(x[-1], y[-1], color='red', label='End Point')

# Plot boundaries
plt.plot(boundary1[:, 0], boundary1[:, 1], 'orange', label='Boundary 1')
plt.plot(boundary2[:, 0], boundary2[:, 1], 'purple', label='Boundary 2')

plt.title("Reference Trajectory and Boundaries")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
