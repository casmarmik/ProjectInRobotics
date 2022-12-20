import socket
import xml.etree.ElementTree as et
import numpy as np
from scipy.spatial import distance as dis
from collections import deque
from time import time, sleep
import rospy as ro
from trajectory_msgs.msg import JointTrajectory as JT
from trajectory_msgs.msg import JointTrajectoryPoint as JTp


# Global variables

class simROS:
    def __init__(self):
        ro.init_node('YELLER', anonymous=True, disable_signals=True)
        self.pub = ro.Publisher('POSES', JT, queue_size=10)
        self.rate = ro.Rate(10) # 10hz publisher rate
        
        self.running   = False
        self.publishableJT = JT()
        self.simPoses = JT()


    def talker(self):
        self.pub.publish(self.publishableJT)
        self.rate.sleep()

    def getPublishablePoses(self):
        return self.publishableJT
    #     l = []
    #    for pos in self.simPoses.points:
    #        l.append(pos.positions)
    #
    #    return np.array(l)
    
    def setPublishablePoses(self, poses, time_stamps):
        for i in range(0, len(time_stamps)):
            print("---------------p and t---------------")
            print(poses[i])
            print(time_stamps[i])
            publishableJTp = JTp()
            publishableJTp.positions = poses[i]
            d = ro.Duration.from_sec(time_stamps[i])
            publishableJTp.time_from_start = d
            print("---------------JTp-------------------")
            print(publishableJTp)
            self.publishableJT.points.append(publishableJTp)
            print("---------------JT-------------------")
            print(self.publishableJT)

if __name__ == '__main__':
    sro = simROS()
    pathPosesForComm = list([[  0.0,0.0,0.0,0.0,0.0,0.0],
                             [41.09, -25.18 +90, 97.35 - 90, 0.0, 17.80, -48.91]])

    time_stamps = [0.0, 10.0]

    print(pathPosesForComm)
    sro.setPublishablePoses(pathPosesForComm, time_stamps)
    print(sro.getPublishablePoses())

    while not ro.is_shutdown():
        try:
            sro.talker()
        except ro.ROSInterruptException:
            pass