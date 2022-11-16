import socket
import xml.etree.ElementTree as et

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
HOST = "127.0.0.1"#"192.168.1.100"
PORT = 4434 #49152
UDP_INFO = (HOST, PORT)
s.bind(UDP_INFO)

IPOC = -1
Gripper = -1
StopFlag = -1

while True:
    print("Listening...")
    xml_encoded, addr = s.recvfrom(1024)
    xml_decoded = xml_encoded.decode('utf-8')

    print("Got something!")

    root = et.fromstring(str(xml_decoded))
    sIPOC = root.find('IPOC').text
    curIPOC = int(sIPOC)
    sGripper = root.find('Gripper').text
    Gripper = int(sGripper)
    sStopFlag = root.find('StopFlag').text
    StopFlag = int(sStopFlag)

    if IPOC != curIPOC:
        IPOC = curIPOC
        print("Current IPOC number: " + str(IPOC))
        print("Current Gripper: " + str(Gripper))
        print("Current StopFlag: " + str(StopFlag) + "\n\n")
        print("Address:" + str(addr))