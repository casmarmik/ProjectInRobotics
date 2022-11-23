import socket
import xml.etree.ElementTree as et
import threading
import time

Gripper = float(0)
StopFlag = float(0)

def buildXMLString(curIPOC):

    xml_ = []
    xml_.append('<Sen Type="ImFree">')
    xml_.append('<Gripper>')
    xml_.append(str(Gripper))
    xml_.append('</Gripper>')
    xml_.append('<StopFlag>')
    xml_.append(str(StopFlag))
    xml_.append('</StopFlag>')
    xml_.append('<IPOC>')
    xml_.append(str(curIPOC))
    xml_.append('</IPOC>')
    xml_.append('</Sen>')

    xml = ""
    for x in xml_:
        xml += x

    return xml

def sendMessagesToRobot():

    print("[Thread]: starting")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    HOST = "192.168.1.102"
    PORT = 49152
    UDP_INFO = (HOST, PORT)
    s.bind(UDP_INFO)

    while True:
        #print("Listening...")
        xml_encoded, addr = s.recvfrom(1024)
        xml_decoded = xml_encoded.decode('utf-8')

        #print("Got something from IP: " + str(addr[0]) + " and port " + str(addr[1]))

        root = et.fromstring(str(xml_decoded))
        #print("Got this:" + xml_decoded)
        sStopFlag = root.find('StopFlag').text
        sIPOC = root.find('IPOC').text
        curIPOC = int(sIPOC)
        #print("Current IPOC number: " + str(curIPOC))

        xml = buildXMLString(curIPOC)

        #print("Sending:" + xml)

        s.sendto(xml.encode('utf-8'), addr)
        
        if sStopFlag == '1.0':
            print("[Thread]: Stopping message sending")
            s.close()
            return

if __name__ == "__main__":
    command = ''
    thr = threading.Thread(target=sendMessagesToRobot)
    thr.start()
    while command != 'q':
        command = input("Press '1' to open gripper, press '2' to close gripper, press 'q' to quit\n")
        if command == '1':
            print("Opening gripper!")
            Gripper = float(1)
        elif command == '2':
            print("Closing Gripper!")
            Gripper = float(0)
        elif command == 'q':
            print("Shutting down...")
            StopFlag = float(1)
            time.sleep(5)
            break
    # This doesn't work, because s.recv is blocking, so the thread will never finish...
    thr.join()