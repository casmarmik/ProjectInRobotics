import numpy as np
import matplotlib.pyplot as plt

def plotPathChanges(path):
    for i in range(0, 6):
        plt.plot(path[:,i])
    
    plt.ylabel('The joint angles')
    plt.xlabel('The number of the samples')
    plt.show()

def interpolatePath(recvPoses, time, cycleRate):
    path = []                                                  # Initialize path list
    lastIndex = len(recvPoses) - 1
    for i in range(0, lastIndex):
        p1 = recvPoses[i,:]                                    # Pose 1, start pose of interpolation
        p2 = recvPoses[i + 1,:]                                # Pose 2, end pose of interpolation 
        pd = p2 - p1                                           # Difference of poses
        td = time[i + 1] - time[i]                             # Difference of timestamps
        nrPoses = int(td/cycleRate)                            # Number of interpolations

        r = np.divide(pd,td)                                   # Slope
        
        
        path.append(p1)
        for j in range(0, nrPoses):                            # Interpolate path
            path.append(r*cycleRate*(j + 1) + p1 )             # Calculate interpolation        
    
    path.append(recvPoses[lastIndex,:])
        
    return np.array(path)

#-----------------------------------------------------------------------------------------------------------------

recvPoses = np.array([[-90.0,90.0,45.0,0.0,0.0,0.0],      # Poses "recved" from simulation, to
                      [-80.0,80.0,30.0,0.0,0.0,0.0],      # be interpolated for robot control
                      [-70.0,50.0,30.0,0.0,0.0,0.0],
                      [-60.0,45.0,30.0,0.0,0.0,0.0]])
timeStamps = [1.0, 2.0, 3.0, 4.0]                         # Simulation timestamps for recvPoses
cycleRate = 0.004                                         # Sensor cycle rate on the robot


path = interpolatePath(recvPoses, timeStamps, cycleRate)
plotPathChanges(path)