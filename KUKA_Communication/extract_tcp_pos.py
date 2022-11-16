import socket
import threading
import time
import math
from xml.etree import ElementTree as ET
from scipy.spatial.transform import Rotation as R

TEST_XML = """
<Rob Type="KUKA">
    <RIst X="390.0" Y="0.0" Z="670.0" A="-180.0" B="0.0" C="180.0"/>
</Rob>
""".strip()
print(TEST_XML)

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

        f = open("vision/data/calibration/tcp.txt", "a")
        f.write(f"{self.tcp_pose[0]},{self.tcp_pose[1]},{self.tcp_pose[2]},{self.tcp_pose[3]},{self.tcp_pose[4]},{self.tcp_pose[5]}\n")
        f.close()

    def read_thread(self):
        self.running = True
        while self.running:
            xml_encoded, _ = self.socket.recvfrom(1024)

            if self.collect_tcp:
                # Parse xml data
                xml_decoded = xml_encoded.decode('utf-8')
                xml = ET.XML(xml_decoded)
                children = xml.getchildren()

                # Find correct child and get tcp coordinates
                for i in range(len(children)):
                    if children[i].tag == "RIst":
                        for j in range(len(self.tcp_pose)):
                            self.tcp_pose[j] = float(children[i].items()[j][1])

                        # Convert from euler angles to rotation vector
                        rot = R.from_euler("ZYX", [math.radians(self.tcp_pose[3]), math.radians(self.tcp_pose[4]), math.radians(self.tcp_pose[5])])
                        rotvec = rot.as_rotvec()
                        self.tcp_pose[3] = rotvec[0]
                        self.tcp_pose[4] = rotvec[1]
                        self.tcp_pose[5] = rotvec[2]

                        print(self.tcp_pose)
                        self.collect_tcp = False
                        break # No need to loop through the other children


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