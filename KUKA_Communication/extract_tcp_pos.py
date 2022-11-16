from pickle import FALSE
import socket
import threading
import time
from xml.etree import ElementTree as ET

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
        self.collect_tcp = FALSE

    def connect(self):
        self.thread.start()
        self.tcp_pose = [0,0,0,0,0,0]
        return
        self.socket.bind((self.host, self.port))
        

    def disconnect(self):
        self.running = False
        self.thread.join()
        return
        self.socket.close()

    def collect_tcp_pose(self):
        self.collect_tcp = True
        # Wait for TCP position to be collected
        while self.collect_tcp == False:
            time.sleep(0.1)
        
        return self.tcp_pose

    def read_thread(self):
        self.running = True
        while self.running:
            
            # Parse xml file
            xml = ET.XML(TEST_XML)
            children = xml.getchildren()

            # Find correct child and get tcp coordinates
            for i in range(len(children)):
                if children[i].tag == "RIst" and self.collect_tcp:
                    for i in range(len(self.tcp_pose)):
                        self.tcp_pose[i] = children[i].items()[i][1]
                        print(self.tcp_pose)
                    self.collect_tcp = False
            continue
            xml_encoded, addr = self.socket.recvfrom(1024)
            xml_decoded = xml_encoded.decode('utf-8')
            print(xml_decoded)

if __name__ == '__main__':
    socket = TCPSocket()
    socket.connect()
    time.sleep(10)
    socket.disconnect()