import socket
import threading
import xml.etree.ElementTree as et
import numpy as np
from scipy.spatial import distance as dis
from collections import deque

# NOTES:
# You must have a static IP-address on your PC that is in the same subnet as the controllers RSI-interface.
# The recommended IP and subnet-mask is "192.168.1.102" and "255.255.0.0"

class controllerComm:
    # IP and port variables are the IP-address and port number of the PC that the controller is sending its data-structure to.
    # IP and port variables must be the same as what's defined in the .XML configuration file on the robot controller,
    # and the IP variable must be the same as the IP-address on the PC from which the script is run.
    def __init__(self, IP="192.168.1.102", port=49152):
        
        self.IP        = IP
        self.port      = port

        # Initialize socket
        self.socket    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Initialize communication thread
        # Argument to thread is the name of the function which is run in the thread
        self.running   = False
        self.thread    = threading.Thread(target=self.communicate)

        # Initialise variable which the PC sends to the robot
        # [String] exitFlag: Must be either '0' (False) or '1' (True).
        # If true, then the program running on the controller is terminated
        # Note, if the controller is currently moving along a path, the stopPath-flag must also be set to true
        self.exitFlag = '0'
        # [String] pathState: Must be either '0' (False) or '1' (True).
        # If false, then the controller executes the path, otherwise it stops executing the current path, if there is one.
        self.pathState = '1'

        self.PTPState = '0'

        self.robotReadyForData = '0'

        self.curRobotAngles = []

        # [Int] gripperState:
        # If gripperState is '0', the controller will open the gripper.
        # Any other positive value larger than zero will close the gripper
        self.gripperState = '0'
        # [Queue] pathPoses: Queue of poses in a path
        # Each array in the queue has the structure [A1,A2,A3,A4,A5,A6], which are the six joint angles of the robot,
        # with A1 being the base joint of the robot. The path is interpolated to satisfy controller contraints.
        self.pathPoses = deque()
        # [Queue] PTPPoses: Queue of poses, which are not in a path.
        # Each array in the queue has the structure [A1,A2,A3,A4,A5,A6], which are the six joint angles of the robot,
        # with A1 being the base joint of the robot. The path is interpolated to satisfy controller contraints.
        self.PTPPoses = deque()
        # [Int] IPOC:
        # Timestamp received from controller. The Pc must send a response witht the same timestamp as the message just received
        self.IPOC = '0'

    # Method for printing debug-messages in the communication-class
    def debugMsg(msg):
        print('[Communication]: ' + msg)        

    def isRunning(self):
        return self.running

    # Setter-methods for data sent to the controller
    def exitProgram(self):
        self.exitFlag = '1'

    def startPath(self):
        self.pathState = '0'

    def startPTP(self):
        self.PTPState = '1'

    def stopPath(self):
        self.pathState = '1'
        self.pathPoses.clear()

    def openGripper(self):
        self.gripperState = '0'

    def closeGripper(self):
        self.gripperState = '1'

    # Starts the socket and thread
    # Parameter timeout is the time in seconds that the PC will wait for a response from the controller
    def start(self, timeout = 60):
        self.socket.bind((self.IP, self.port))
        self.socket.settimeout(timeout)
        self.thread.start()
        self.debugMsg("Communication started")

    # Terminates the thread and socket
    def close(self):
        self.thread.join()
        self.socket.close()
        self.debugMsg("Communication closed")

    # Returns the XML-string of the variables formatted such that the controller can receive them
    def buildXMLString(self):
        
        # Start tag
        xml = '<Sen Type="ImFree">'

        # Exit flag
        xml += '<ExitFlag>' + self.exitFlag + '</ExitFlag>'

        # Path state
        xml += '<PathState>' + self.pathState + '</PathState>'

        # Stop Path
        xml += '<StopPath>' + self.pathState + '</StopPath>'

        # PTP state
        xml += '<PTPState>' + self.PTPState + '</PTPState>'

        # Gripper state
        xml += '<Gripper>' + self.gripperState + '</Gripper>'
        
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

    def addPathPose(self, pose):
        self.pathPoses.append(pose)

    def addPTPPose(self, pose):
        self.PTPPoses.append(pose)

    def getPoseInXML(self, jointAngles):
        poseXML = ""
        tags = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        poseXML += '<Angles'
        for i in range(0, len(jointAngles)):
            poseXML += ' ' + tags[i] + '="' + str(jointAngles[i]) + '"'
        poseXML += '/>'
        return poseXML

    def getPathPose(self):
        if self.pathPoses.count > 0 and self.pathState == '0' and self.robotReadyForData == '1':
            tempPose = self.getPoseInXML(self.pathPoses.popleft())
            if self.pathPoses.count == 0:
                self.pathState = '1'
            return tempPose
        else:
            return self.getPoseInXML(self.curRobotAngles)

    def getPTPPose(self):
        if self.PTPPoses.count > 0 and self.PTPState == '1': 
            tempPose = self.getPoseInXML(self.PTPPoses[0])
            if self.robotReadyForData == '1':
                self.PTPPoses.pop()
                self.PTPState == '0'
            return tempPose
        else:
            return self.getPoseInXML(self.curRobotAngles)

    # Check if the actual pose of the robot matches the PTP pose the robot should move to, within a specified threshold
    # If it does, return true, otherwise return false. Also returns the euclidian distance 
    def checkPTPPose(self, errorThreshold=0.01):
        if self.PTPPoses.count == 0:
            return False, -1
        else:
            compDist = dis.euclidean(self.getPTPPose(), self.curRobotAngles)
            if compDist > errorThreshold:
                return False, compDist
            else:   
                return True, compDist
    
    def communicate(self):
        self.running = True

        while self.running:
            
            # Receive xml message
            xmlEncoded, addr = self.socket.recvfrom(1024)

            # Parse xml message
            xmlDecoded = xmlEncoded.decode('utf-8')
            xml = et.fromstring(str(xmlDecoded))

            # Get relevant message info
            self.IPOC = xml.find('IPOC').text
            self.robotReadyForData = xml.find('readyForData').text
            self.curRobotAngles = self.getCurrentRobotPoseAngles(xml)
            
            ## Logic starts here

            # TODO: Secure 4 ms between each message

            # Compare ptp pose currently heading to with actual pose
            #comp, compDist = self.checkPTPPose()
            #if comp:
                #adjPoseXML = self.getPTPPoseInXML()
            
            # Adjust til they are within the set error threshold
            XML_MSG = self.buildXMLString()

            if self.exitFlag == '1':
                self.running = False
            self.socket.sendto(XML_MSG.encode('utf-8'), addr)
         
if __name__ == '__main__':
    
    # Message prefix for debugging print statements
    msgPrefix = '[Main ]: '

    # Initialize instance of controllerComm object with default parameters
    comm = controllerComm()
    # Setup ptp poses
    comm.setStartAndEndPoses()
    

    # Start the communication
    comm.start()
    print("(0-0) {Connected! Thread up and running!]")

    c_com = True
    while True:
        command = input("Press 'q' to quit: ")

#        if c_com == True:
#            command = input("Press '1' to open gripper, press '2' to close gripper, press c to MOVE robot or press 'q' to quit: ")
#        else:
#            command = input("Press '1' to open gripper, press '2' to close gripper, press c to STOP robot or press 'q' to quit: ")

        # if command == '1':
        #     print("Opening gripper!")
        #     s.sGripper = '1.0'
            
        # elif command == '2':
        #     print("Closing Gripper!")
        #     s.sGripper = '0.0'
            
#        elif command == 'c':
#            c_com = s.ptpMove(c_com)
            
        # Desired commands:
        # 1. Move home command
        # 2. Quit-command (already have this)
        # 3. Move path
        # 4. Stop path
        # 5. Actuate gripper
        # 6. Move PTP

        # Suggestion for main commands for PTP-movement
        #comm.addPTPPose()
        #comm.startPTP()
        #while comm.PTPState == '1':
        #    continue
        #while comm.robotReadyForData == '1':
        #    continue


        if command == 'q':
            print("(=_0)>{Shutting down...]")
            comm.stopPath()
            comm.exitProgram()
            break
        else:
            print("unknown input please retry!")
    
    while comm.isRunning():
        continue
    comm.close()