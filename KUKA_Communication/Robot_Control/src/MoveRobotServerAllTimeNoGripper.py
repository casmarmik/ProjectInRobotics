import socket
import threading
import xml.etree.ElementTree as et
import numpy as np
from scipy.spatial import distance as dis


class TCPSocket:
    def __init__(self, host="192.168.1.102", port=49152):
        self.host      = host
        self.port      = port
        self.socket    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running   = False
        self.thread    = threading.Thread(target=self.read_thread)

        self.actPose   = [0,0,0,0,0,0]
        self.collecAct = False

        self.npPTPPose = np.array([[0,0,0,0,0,0],
                                   [0,0,0,0,0,0],
                                   [0,0,0,0,0,0],
                                   [0,0,0,0,0,0]])
        self.sGripper  = '0.0'
        self.sStopFlag = '0.0'
        self.sNoMoFlag = '1.0'
        
        self.poseI     = 0

    def connect(self, timeout):
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(timeout)
        self.thread.start()

    def disconnect(self):
        self.running = False
        print("(=_0)zzZZ")
        self.thread.join()
        print("(=_=)zzZZ")
        self.socket.close()
        print(" ___")
        print("Shutdown complete")

    def buildXMLString(self, adjPoseXML):
        xml_ = []
        xml_.append('<Sen Type="ImFree">')
        
        # Flag for stopping all process and shutting down
        xml_.append('<StopFlag>')
        xml_.append(self.sStopFlag)
        xml_.append('</StopFlag>')
        
        # Entire XML line descriping the desired pose to ptp to
        xml_.append(adjPoseXML)
        
        # IPOC from revieved msg
        xml_.append('<IPOC>')
        xml_.append(self.sIPOC)
        xml_.append('</IPOC>')

        xml_.append('</Sen>')

        xml = ""
        for x in xml_:
            xml += x

        return xml
    
    def getActualPose(self, xml):
        actPose = []
        actPose_tag = xml.find('RIst')
        actPose.append(float(actPose_tag.attrib['X']))
        actPose.append(float(actPose_tag.attrib['Y']))
        actPose.append(float(actPose_tag.attrib['Z']))
        actPose.append(float(actPose_tag.attrib['A']))
        actPose.append(float(actPose_tag.attrib['B']))
        actPose.append(float(actPose_tag.attrib['C']))

        return actPose

    def setPTPPoses(self):
        self.npPTPPoses = np.array([[0,0,0,0,0,0],
                                    [0,0,0,0,0,0],
                                    [0,0,0,0,0,0],
                                    [0,0,0,0,0,0]])

    def getPTPPoseInXML(self):
        poseXML = []
        tags = ['X', 'Y', 'Z', 'A', 'B', 'C']
        pose = self.npPTPPoses[self.poseI]
        poseXML.append('<RIst')
        for i in range(0, len(pose)):
            poseXML.append(' ' + tags[i] + '="' + str(pose[i]) + '"')
        poseXML.append('/>')

        sPoseXML = ""
        for x in poseXML:
            sPoseXML += x
        
        return sPoseXML
    
    def comparePoses(self, errorThreshold):
        compDist = dis.euclidean(self.npPTPPoses[self.poseI], self.actPose)
        if compDist > errorThreshold:
            return False, compDist
        else:   
            return True, compDist
    
    def read_thread(self):
        self.running = True
        while self.running:
            if self.poseI >= len(self.npPTPPoses):
                self.sStopFlag = '1.0'
                
            # Incase massages should stop
            if self.sStopFlag == '1.0':
                print("[Thread]: Stopping sending messages")
                return
            
            # Recv xml msg
            xmlEncoded, addr = self.socket.recvfrom(1024)

            # Parse xml msg
            xmlDecoded = xmlEncoded.decode('utf-8')
            xml = et.fromstring(str(xmlDecoded))

            # Get relevant msg info
            #sStopFlag = xml.find('StopFlag').text
            self.sIPOC = xml.find('IPOC').text
            self.actPose = self.getActualPose(xml)
            
            # Set error that is allowed between poses
            errorThreshold = 0.01
            # Compare ptp pose currently heading to with actual pose
            comp, compDist = self.comparePoses(errorThreshold)
            if comp:
                self.poseI += 1
            
            # Adjust til they are within the set error threshold
            adjPoseXML = self.getPTPPoseInXML()
            adjXMLMSG = self.buildXMLString(adjPoseXML)
            self.socket.sendto(adjXMLMSG.encode('utf-8'), addr)
            print("(0-0) {IPOC: " + self.sIPOC + "]")
            print("(0-0) {Actual Pose: " + str(self.actPose) + "]")
            print("(0-0) {Desired Pose: " + str(self.npPTPPose[self.poseI]) + "]")
            print("(0-0) {Pose Reached: " + str(comp) + "]")
            print("(0-0) {Pose nr. sending: " + str(self.poseI) + "]")
        
        
if __name__ == '__main__':
    # Setup socket
    s = TCPSocket(host="localhost", port=4434)
    print("(=-=) {Initialization proces...]")
    # Setup ptp poses
    s.setPTPPoses()
    print("(0-=) {complete]")
    

    # connect socket
    s.connect(10)
    print("(0-0) {Connected! Thread up and running!]")

    running = True
    c_com = True
    while running:
        command = input("The robot is recieving poses and moves accordingly at this moment, press 'q' to quit: ")

        if command == 'q':
            print("(=_0)>{Shutting down...]")
            s.sStopFlag = '1.0'
            break
        else:
            print("unknown input please retry!")
    
    s.disconnect()