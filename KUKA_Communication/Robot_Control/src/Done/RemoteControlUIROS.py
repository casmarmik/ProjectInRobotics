import socket
import threading
import xml.etree.ElementTree as et
import numpy as np
from scipy.spatial import distance as dis
from collections import deque
from time import time, sleep
import rospy as ro
from trajectory_msgs.msg import JointTrajectory as JT
from trajectory_msgs.msg import JointTrajectoryPoint as JTp

# Global variables
currPoseForPublisher = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class simROS:
    def __init__(self):
        ro.init_node('listener', anonymous=True, disable_signals=True)
        self.running   = False
        self.publishableJT = JT()
        self.simPoses = JT()
        self.pub = None
        self.rate = None

        try:
            self.whisper()
        except ro.ROSInterruptException:
            pass

        try:
            self.listener()
        except ro.ROSInterruptException:
            pass

        self.thread   = threading.Thread(target=self.talker)
    
    def talker(self):
        global currPoseForPublisher 
        while self.running:
            publishableJTp = JTp()
            publishableJTp.positions = currPoseForPublisher
            self.publishableJT.points.append(publishableJTp)
            self.pub.publish(self.publishableJT)
            self.rate.sleep()

    def startPublisher(self):
        self.running = True
        self.thread.start()

    def stopPublisher(self):
        self.running = False

    def getNPPoses(self):
        l = []
        l.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        home = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
        for pos in self.simPoses.points:
            t_temp = pos.time_from_start.to_sec()
            temp = []
            for i in range(6):
                temp.append(np.rad2deg(pos.positions[i]) -home[i])
            temp.append(t_temp)
            # print("--------------------------------------")
            # print(temp)
            # print("--------------------------------------")

            l.append(temp)
        
        return np.array(l)

    def getSimPoses(self):
            return self.simPoses

    def getPub(self):
            return self.pub

    def getRate(self):
            return self.rate

    def call(self, jt):
        # Setting the sim_poses to the recieved values
        self.simPoses = jt

    def whisper(self):
        self.pub = ro.Publisher('WHEREIAM', JT, queue_size=10)
        self.rate = ro.Rate(10) # 10hz publisher rate

    def listener(self):
        ro.Subscriber("/POSES", JT, self.call)

    



# NOTES:
# You must have a static IP-address on your PC that is in the same subnet as the controllers RSI-interface.
# The recommended IP and subnet-mask is "192.168.1.102" and "255.255.0.0"

class controllerComm:
    # IP and port variables are the IP-address and port number of the PC that the controller is sending its data-structure to.
    # IP and port variables must be the same as what's defined in the .XML configuration file on the robot controller,
    # and the IP variable must be the same as the IP-address on the PC from which the script is run.
    def __init__(self, IP="192.168.1.102", port=49152):
        #urllib.request.urlopen("https://" + IP + ":" + str(port))
        
        self.IP        = IP
        self.port      = port

        self.timePathStart = 0

        #self.startingPathPose = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]

        # Initialize socket
        self.socket    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Initialize communication thread
        # Argument to thread is the name of the function which is run in the thread
        self.running   = False
        self.thread    = threading.Thread(target=self.communicate)

        # Initialize variables which the PC sends to the robot
        # [Char] exitFlag: Must be either '0' (False) or '1' (True).
        # If true, all motion is stopped and the program running on the controller is terminated
        self.exitFlag = '0'
        # [Char] pathState: Must be either '0' (False) or '1' (True).
        # If true, then the controller executes the path, otherwise it stops executing the current path, if there is one.
        self.pathState = '0'
        # [Char] pathState: Must be either '0' (False) or '1' (True).
        # If true, then the controller executes the path, otherwise it stops executing the current path, if there is one.
        self.PTPState = '0'
        # Stop block in RSI-diagram is triggered on rising edge, meaning when this variables goes from 0 to 1
        self.stopPath = '0'

        self.robotIsShutdown = '0' # If '1', then robot is done shutting down
        self.robotPathDone = '1'

        # [Array] curRobotAngles: Current joint angles of the robot
        self.curRobotAngles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # [Char] gripperState:
        # If gripperState is '0', the controller will close the gripper.
        # Any other positive value larger than zero will open the gripper
        self.gripperState = '0'
        # [Queue] pathPoses: Queue of poses in a path
        # Each array in the queue has the structure [A1,A2,A3,A4,A5,A6], which are the six joint angles of the robot,
        # with A1 being the base joint of the robot. The path is interpolated to satisfy controller contraints.
        self.pathPoses = deque()
        # [Queue] PTPPoses: Queue of poses, which are not in a path.
        # Each array in the queue has the structure [A1,A2,A3,A4,A5,A6], which are the six joint angles of the robot,
        # with A1 being the base joint of the robot.
        self.PTPPoses = deque()
        # [String] IPOC:
        # Timestamp received from controller. The PC must send a response with the same timestamp as the message just received
        self.IPOC = '0'

    # Method for printing debug-messages in the communication-class
    def debugMsg(self, msg):
        print('[Thread]: ' + msg)        

    def isRunning(self):
        return self.running

    # Setter-methods for data sent to the controller
    def exitProgram(self):
        self.stopMotion()
        self.exitFlag = '1'

    # The poses-variable can be a 2D np-array of multiple poses, 
    # or a single pose in the form of a list containing six joint angles
    def startPath(self, poses):
        #self.debugMsg("Starting path pose is now: " + str(self.curRobotAngles[0]) + str(self.curRobotAngles[1]) + str(self.curRobotAngles[2]) + str(self.curRobotAngles[3]) + str(self.curRobotAngles[4]) + str(self.curRobotAngles[5]))
        #self.startingPathPose = self.curRobotAngles
        self.pathPoses.clear()
        self.stopPath = '0'
        self.pathState = '1'
        while self.robotPathDone == '1':
            sleep(0.1)
        self.addPosesToQueue(poses,'Path', sro)
        print("(=<>=)HELLO!")
        self.timePathStart = time()

    # The poses-variable can be a 2D np-array of multiple poses, 
    # or a single pose in the form of a list containing six joint angles
    def startPTP(self, poses):
        self.addPosesToQueue(poses,'PTP', sro)
        self.PTPState = '1'

    def stopMotion(self):
        self.pathState = '0'
        self.PTPState = '0'
        self.stopPath = '1'
        self.pathPoses.clear()
        self.PTPPoses.clear()

    def openGripper(self):
        self.gripperState = '1'

    def closeGripper(self):
        self.gripperState = '0'

    # Starts the socket and thread
    # Parameter timeout is the time in seconds that the PC will wait for a response from the controller
    def start(self, timeout = 6000000):
        self.socket.bind((self.IP, self.port))
        self.socket.settimeout(timeout)
        self.thread.start()
        self.debugMsg("Communication started")

    # Terminates the thread and socket
    def close(self, sro):
        sro.stopPublisher()
        sro.thread.join()
        self.thread.join()
        self.socket.close()
        ro.signal_shutdown("reason") 
        self.debugMsg("Communication closed")

    # Returns the XML-string of the variables formatted such that the controller can receive them
    def buildXMLString(self):
        
        # Start tag
        xml = '<Sen Type="ImFree">'

        # Exit flag
        xml += '<ExitFlag>' + self.exitFlag + '</ExitFlag>'

        # Path state
        xml += '<PathState>' + self.pathState + '</PathState>'

        # PTP state
        xml += '<PTPState>' + self.PTPState + '</PTPState>'

        # Gripper state
        xml += '<Gripper>' + self.gripperState + '</Gripper>'

        # Path state start/stop flag to RSI-block
        xml += '<PathStateStop>' + self.stopPath + '</PathStateStop>'
        
        # Path pose
        xml += self.getPathPose()

        # PTP pose
        xml += self.getPTPPose()
        
        # IPOC timestamp
        xml += '<IPOC>' + self.IPOC + '</IPOC>'

        # End tag
        xml += '</Sen>'

        return xml
    
    # Returns the current pose of the robot as list of 6 joint angles.
    # The first value is the joint angle of the base joint of the robot, and so on.
    def getCurrentRobotPoseAngles(self, xml):
        actPose = []
        actPose_tag = xml.find('AIPos')
        actPose.append(float(actPose_tag.attrib['A1']))
        actPose.append(float(actPose_tag.attrib['A2']))
        actPose.append(float(actPose_tag.attrib['A3']))
        actPose.append(float(actPose_tag.attrib['A4']))
        actPose.append(float(actPose_tag.attrib['A5']))
        actPose.append(float(actPose_tag.attrib['A6']))
        return actPose

    def addPosesToQueue(self, poses, queueType, sro):
        # If there is more than one pose
        print("(=<>=)HELLO!")
        # print(poses)
        if type(poses) is np.ndarray:
            for pose in poses:
                if queueType == 'PTP':
                    self.PTPPoses.append(pose)
                else:
                    self.pathPoses.append(pose)
        else:
            if queueType == 'PTP':
                self.PTPPoses.append(poses)
            else:
                self.pathPoses.append(poses)

    def getPoseInXML(self, jointAngles, type):
        poseXML = ""
        tags = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        poseXML += '<' + type + 'Angles'
        for i in range(0, len(jointAngles)):
            poseXML += ' ' + tags[i] + '="' + str(jointAngles[i]) + '"'
        poseXML += '/>'
        return poseXML

    def getPathPose(self):
        #self.debugMsg("Path Done: " + self.robotPathDone)
        if len(self.pathPoses) > 0 and self.robotPathDone == '0':
            timeElapsed = time() - self.timePathStart
            if len(self.pathPoses) > 1 and self.pathPoses[1][-1] < timeElapsed:
                self.debugMsg("Current Time: " + str(timeElapsed))
                self.debugMsg("Path poses: " + str(self.pathPoses))
 
                self.pathPoses.popleft()
            if len(self.pathPoses) == 1:
                self.pathState = '0'
                self.stopPath = '1'
                return self.getPoseInXML(self.pathPoses[0][:-1], 'Path')    
            else:   
                startPoint = self.pathPoses[0]
                endPoint = self.pathPoses[1]
                jointAngles = interpolatePath(startPoint, endPoint, timeElapsed)
                nextPose = self.getPoseInXML(jointAngles, 'Path')
                return nextPose
        else:
            return self.getPoseInXML([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'Path')

    def getPTPPose(self):
        if len(self.PTPPoses) > 0 and self.PTPState == '1':
            # Only remove pose from queue, when it is near current robot joint angles
            nextPose = self.getPoseInXML(self.PTPPoses[0], 'PTP')
            isIdentical, _ = self.matchesCurPose(self.PTPPoses[0])
            if isIdentical:
                self.PTPPoses.popleft()
            return nextPose
        else:
            # Reset PTP-state and send current robot joint angles
            self.PTPState = '0'
            return self.getPoseInXML(self.curRobotAngles, 'PTP')

    # Check if the actual pose of the robot matches the PTP pose the robot should move to, within a specified threshold
    # If it does, return true, otherwise return false. Also returns the euclidian distance 
    def matchesCurPose(self, pose, errorThreshold=0.01):
        compDist = dis.euclidean(pose, self.curRobotAngles)
        if compDist > errorThreshold:
            return False, compDist
        else:   
            return True, compDist

    def communicate(self):
        self.running = True

        while self.running:

            if self.robotIsShutdown == '1':
                self.debugMsg("Shutting down")
                self.running = False
                break

            # Receive xml message

            try:
                xmlEncoded, addr = self.socket.recvfrom(1024)
            except:
                self.debugMsg("Timed out waiting for controller response")
                self.running = False
                break

            # Parse xml message
            xmlDecoded = xmlEncoded.decode('utf-8')
            xml = et.fromstring(str(xmlDecoded))

            # Get relevant message info
            self.IPOC = xml.find('IPOC').text
            self.robotPathDone = xml.find('robotPathDone').text
            self.robotIsShutdown = xml.find('robotIsShutdown').text
            self.curRobotAngles = self.getCurrentRobotPoseAngles(xml)
            
            global currPoseForPublisher
            currPoseForPublisher = self.curRobotAngles
            
            # Adjust til they are within the set error threshold
            XML_MSG = self.buildXMLString()
     

            self.socket.sendto(XML_MSG.encode('utf-8'), addr)





# path is a 2D numpy array, where each array conatins the six joint angles,
# and the 7'th value is the timestamp
# cycleTime is in s
def interpolatePath(startPoint, endPoint, timeSincePathStart):
    # print(startPoint)
    # print(endPoint)
    # for i in range(6):
    #     startPoint[i] = np.rad2deg(startPoint[i]) - home[i]
    #     endPoint[i] = np.rad2deg(endPoint[i]) - home[i]
    pd = endPoint[:-1] - startPoint[:-1]
    td = endPoint[-1] - startPoint[-1]
    curTime = timeSincePathStart - startPoint[-1]
    slope = np.divide(pd,td)
    return slope * curTime + startPoint[:-1]


# MAIN CODE ----------------------------------------------------------------------
if __name__ == '__main__':
    
    # Message prefix for debugging print statements
    msgPrefix = '[Main ]: '

    # Default joint angles values for robot home position
    # Robot will go to here when the home function is called
    home = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]

    # Initialize instance of controllerComm object with default parameters
    comm = controllerComm()
    # Setup ptp poses
    #comm.setStartAndEndPoses()


    #publishable_pose = np.array([[0.0,0.0,0.0,0.0,0.0,0.0,0.0],
    #                             [-10.0,0.0,0.0,0.0,0.0,0.0,4.0],
    #                             [-20.0,0.0,0.0,0.0,0.0,0.0,8.0],
    #                             [-30.0,0.0,0.0,0.0,0.0,0.0,12.0]])

    # Recieve Path poses from ROS node ----- for now hard coded as a reversed poses list
    #pathPosesForComm = np.array([[0.0,0.0,0.0,0.0,0.0,0.0,0.0],
    #                            [-10.0,0.0,0.0,0.0,0.0,0.0,4.0],
    #                            [-20.0,0.0,0.0,0.0,0.0,0.0,8.0],
    #                            [-30.0,0.0,0.0,0.0,0.0,0.0,12.0]])
    # Set PTP motions where needed ----- for now hard coded as a reversed poses list
    PTPPosesForComm = np.array([[10.0,-90.0,90.0,0.0,0.0,0.0],
                                [20.0,-90.0,90.0,0.0,0.0,0.0],
                                [30.0,-90.0,90.0,0.0,0.0,0.0],
                                [40.0,-90.0,90.0,0.0,0.0,0.0]])

    # Start the communication
    comm.start(30)
    print(msgPrefix + 'Spinning up user menu')

    # Start up user interface for the user
    commandInfo = '''
    Options: 
    Press 'm' to move along an interpolated path.
    Press 'p' to move between pose(s) using PTP movement.
    Press 's' to stop movements.
    Press 'h' to return to home position.
    Press 'o' to open the gripper.
    Press 'c' to close the gripper.
    Press 'q' to quit the program.
    '''
    sro = simROS()
    sro.startPublisher()

    while not ro.is_shutdown():
        print(currPoseForPublisher)
        command = input(commandInfo)
        if command == 'p':
            print("Starting PTP motion")
            comm.startPTP(PTPPosesForComm)
        elif command == 'h':
            print("Heading home!")
            comm.startPTP(home)
        elif command == 'm':
            print("Starting path motion")
            comm.startPath(sro.getNPPoses())
        elif command == 's':
            print(msgPrefix + "Stopping motion")
            comm.stopMotion()
        elif command == 'o':
            print(msgPrefix + "Opening Gripper!")
            comm.openGripper()
        elif command == 'c':
            print(msgPrefix + "Closing gripper!")
            comm.closeGripper()
        elif command == 'q':
            print(msgPrefix + "Shutting down...")
            comm.exitProgram()
            break
        else:
            print(msgPrefix + "Unknown input please retry!")
        print("(-.-)hello")
    while comm.isRunning():
        sleep(0.1)
    comm.close(sro)
    