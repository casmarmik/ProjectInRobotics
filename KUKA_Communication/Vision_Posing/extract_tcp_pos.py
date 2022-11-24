import socket
import threading
import time
import math
from xml.etree import ElementTree as ET
from scipy.spatial.transform import Rotation as R

class TCPSocket:
    def __init__(self, host="192.168.1.102", port=49152):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running = False
        self.thread = threading.Thread(target=self.read_thread)

        self.tcp_pose = None
        self.collect_tcp = False

    def connect(self):
        self.socket.bind((self.host, self.port))
        self.thread.start()
        self.tcp_pose = [0,0,0,0,0,0]

    def disconnect(self):
        self.running = False
        self.thread.join()
        self.socket.close()

    def collect_tcp_pose(self):
        self.collect_tcp = True
        # Wait for TCP position to be collected
        while self.collect_tcp == False:
            time.sleep(0.1)

        if self.tcp_pose[0] == self.tcp_pose[1] == self.tcp_pose[2] == self.tcp_pose[3] == self.tcp_pose[4] == self.tcp_pose[5] == 0:
            return
            
        f = open("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/calibration/tcp.txt", "a")
        f.write(f"{self.tcp_pose[0]},{self.tcp_pose[1]},{self.tcp_pose[2]},{self.tcp_pose[3]},{self.tcp_pose[4]},{self.tcp_pose[5]}\n")
        f.close()

    def read_thread(self):
        self.running = True
        while self.running:
            xml_encoded, addr = self.socket.recvfrom(1024)
            # Parse xml data
            xml_decoded = xml_encoded.decode('utf-8')
            #print(xml_decoded)
            xml = ET.fromstring(str(xml_decoded))

            if self.collect_tcp:
                rist = xml.find("RIst")
                for i in range(len(self.tcp_pose)):
                    self.tcp_pose[i] = float(rist.items()[i][1])

                # Convert from euler angles to rotation vector
                rot = R.from_euler("ZYX", [math.radians(self.tcp_pose[3]), math.radians(self.tcp_pose[4]), math.radians(self.tcp_pose[5])])
                rotvec = rot.as_rotvec()
                self.tcp_pose[3] = rotvec[0]
                self.tcp_pose[4] = rotvec[1]
                self.tcp_pose[5] = rotvec[2]

                print(self.tcp_pose)
                self.collect_tcp = False
                    
            sIPOC = xml.find('IPOC').text
            xmlMsg = self.buildXMLString(0,sIPOC)
            self.socket.sendto(xmlMsg.encode('utf-8'), addr)
        
        # Send stop flag to robot
        xml_encoded, addr = self.socket.recvfrom(1024)
        xml_decoded = xml_encoded.decode('utf-8')
        xml = ET.XML(xml_decoded)
        sIPOC = xml.find('IPOC').text
        xmlMsg = self.buildXMLString(1,sIPOC)
        self.socket.sendto(xmlMsg.encode('utf-8'), addr)

    def buildXMLString(self,StopFlag,curIPOC):
        xml_ = []
        xml_.append('<Sen Type="ImFree">')
        xml_.append('<Gripper>')
        xml_.append("1")
        xml_.append('</Gripper>')
        xml_.append('<StopFlag>')
        xml_.append(str(StopFlag))
        xml_.append('</StopFlag>')
        xml_.append('<IPOC>')
        xml_.append(curIPOC)
        xml_.append('</IPOC>')
        xml_.append('</Sen>')

        xml = ""
        for x in xml_:
            xml += x

        return xml


if __name__ == '__main__':
    # Setup socket
    s = TCPSocket()
    s.connect()
    running = True
    while running:
        command = input("Type e to exit or c to capture tcp position: ")
        if command == 'e':
            break
        elif command == 'c':
            s.collect_tcp_pose()
        else:
            print("unknown input please retry!")

    s.disconnect()