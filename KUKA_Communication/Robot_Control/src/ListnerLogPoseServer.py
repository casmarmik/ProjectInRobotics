import socket
import xml.etree.ElementTree as et
import threading
import time

StopFlag = '0.0'
PoseReachedFlag = '0.0'
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
timeout = 10 # seconds
HOST = "127.0.0.1"
PORT = 4434
UDP_INFO = (HOST, PORT)
s.bind(UDP_INFO)
s.settimeout(timeout)

def buildXMLString(curIPOC):

    xml_ = []
    xml_.append('<Sen Type="ImFree">')
    
    xml_.append('<PoseReachedFlag>')
    xml_.append(PoseReachedFlag)
    xml_.append('</PoseReachedFlag>')
    
    xml_.append('<StopFlag>')
    xml_.append(StopFlag)
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

    while True:
        print("Listening...")
        xml_encoded, addr = s.recvfrom(1024)
        xml_decoded = xml_encoded.decode('utf-8')

        print("Got something from IP: " + str(addr[0]) + " and port " + str(addr[1]))

        root = et.fromstring(str(xml_decoded))
        #print("Got this:" + xml_decoded)
        sStopFlag = root.find('StopFlag').text
        sIPOC = root.find('IPOC').text
        curIPOC = int(sIPOC)
        print("Current IPOC number: " + str(curIPOC))

        xml = buildXMLString(curIPOC)

        #print("Sending:" + xml)
        if sStopFlag == '1.0':
            print("[Thread]: Stopping message sending")
            return
        s.sendto(xml.encode('utf-8'), addr)
        


if __name__ == "__main__":
    command = ''
    thr = threading.Thread(target=sendMessagesToRobot)
    thr.start()
    while command != 'q':
        command = input("Currently listening, press 'q' if you want to quit\n")
        if command == 'q':
            print("Shutting down...")
            StopFlag = '1.0'
            break
        else:
            print("unknown input please retry!")

    thr.join()
    s.close()
