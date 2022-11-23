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
        #self.sGripper  = '0.0'
        self.sStopFlag = '0'
        #self.sNoMoFlag = '1.0'
        self.poseI     = 0

    def connect(self, timeout):
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(timeout)
        self.thread.start()

    def disconnect(self):
        self.running = False
        #print("(=_0)zzZZ")
        self.thread.join()
        #print("(=_=)zzZZ")
        self.socket.close()
        #print(" ___")
        print("Shutdown complete")

    def buildXMLString(self, adjPoseXML):
        xml_ = []
        xml_.append('<Sen Type="ImFree">')

        # Gripper control
        #xml_.append('<Gripper>')
        #xml_.append(self.sGripper)
        #xml_.append('</Gripper>')

        # Flag for stopping robot movement or reversely allow for movement
#        xml_.append('<noMoFlag>')
#        xml_.append(self.sNoMoFlag)
#        xml_.append('</noMoFlag>')
        
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
        actPose_tag = xml.find('AIPos')
        actPose.append(float(actPose_tag.attrib['A1']))
        actPose.append(float(actPose_tag.attrib['A2']))
        actPose.append(float(actPose_tag.attrib['A3']))
        actPose.append(float(actPose_tag.attrib['A4']))
        actPose.append(float(actPose_tag.attrib['A5']))
        actPose.append(float(actPose_tag.attrib['A6']))

        return actPose

    def setPTPPoses(self):
        self.npPTPPoses = np.array([[-44.5,-90.0,90.0,0.0,0.0,0.0],
                                    [-44.5,-134.5,90.0,0.0,0.0,0.0],
                                    [30.5,-62.3,90.0,0.0,0.0,0.0],
                                    [14.9,-62.3,5.4,0.0,0.0,0.0]])

    def getPTPPoseInXML(self):
        poseXML = ""
        tags = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        pose = self.npPTPPoses[self.poseI]
        poseXML += '<Angles'
        for i in range(0, len(pose)):
            poseXML += ' ' + tags[i] + '="' + str(pose[i]) + '"'
        poseXML += '/>'
        
        return poseXML
    
    def comparePoses(self, errorThreshold):
        compDist = dis.euclidean(self.npPTPPoses[self.poseI], self.actPose)
        if compDist > errorThreshold:
            return False, compDist
        else:   
            return True, compDist
    
    def read_thread(self):
        self.running = True
        adjPoseXML = self.getPTPPoseInXML()

        while self.running:
            if self.poseI >= len(self.npPTPPoses):
                self.sStopFlag = '1'
                
            # Incase massages should stop
            if self.sStopFlag >= '1':
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
            print("[Thread]: Actual pose: " + str(self.actPose))
            print("[Thread]: Stop Flag: " + str(self.sStopFlag))
            print("[Thread]: poseI: " + str(self.poseI))
            
            # Set error that is allowed between poses
            errorThreshold = 0.01
            # Compare ptp pose currently heading to with actual pose
            comp, compDist = self.comparePoses(errorThreshold)
            if comp:
                if self.poseI != len(self.npPTPPoses):
                    self.poseI += 1
                adjPoseXML = self.getPTPPoseInXML()
            
            # Adjust til they are within the set error threshold
            
            adjXMLMSG = self.buildXMLString(adjPoseXML)
            self.socket.sendto(adjXMLMSG.encode('utf-8'), addr)
            # print("(0-0) {IPOC: " + self.sIPOC + "]")
            # print("(0-0) {Actual Pose: " + str(self.actPose) + "]")
            # print("(0-0) {Desired Pose: " + str(self.npPTPPose[self.poseI]) + "]")
            # print("(0-0) {Pose Reached: " + str(comp) + "]")
            # print("(0-0) {Pose nr. sending: " + str(self.poseI) + "]")

            
        
#    def ptpMove(self, move):
#        if move:
#            print("(>0-0)>{Motion Active]")
#            self.sNoMoFlag = '0.0'
#            return False
#        else:
#            print("<(0-0<) {Motion Deactive]")
#            self.sNoMoFlag = '1.0'
#            return True
            
        
        
if __name__ == '__main__':
    
    # Setup socket
    s = TCPSocket()
    # Setup ptp poses
    s.setPTPPoses()
    

    # connect socket
    s.connect(60)
    print("(0-0) {Connected! Thread up and running!]")

    running = True
    c_com = True
    while running:
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
            
        if command == 'q':
            print("(=_0)>{Shutting down...]")
            s.sStopFlag = '1'
            break
        else:
            print("unknown input please retry!")
    
    s.disconnect()