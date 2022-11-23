import socket
import time

def xml_sum(xml_):
    xml = ""
    for x in xml_:
        xml += x

    return xml

UDP_INFO = ("127.0.0.1", 4434)

print("UDP target IP: %s" % UDP_INFO[0])
print("UDP target port: %s" % UDP_INFO[1])

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ipoc = 123456123456

while True:
    xml_ = []
    xml_.append('<Rob TYPE="KUKA">\n')
    xml_.append('<Gripper>1</Gripper>\n')
    xml_.append('<StopFlag>0</StopFlag>\n')
    xml_.append('<IPOC>')
    xml_.append(str(ipoc))
    xml_.append('</IPOC>\n')
    xml_.append('</Rob>')
    xml = xml_sum(xml_)
    s.sendto(xml.encode('utf-8'), UDP_INFO)
    ipoc += 1
    time.sleep(1)
    print("Current IPOC:" + str(ipoc))