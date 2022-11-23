#!/usr/bin/env python
# coding: utf-8

# ## Client
# Links: 
# https://stackoverflow.com/questions/2623054/how-to-send-raw-xml-in-python
# https://wiki.python.org/moin/UdpCommunication

# https://www.pythontutorial.net/python-basics/python-read-text-file/

import socket
import time

def xml_sum(xml_):
    xml = ""
    for x in xml_:
        xml += x

    return xml

def xml_writer(xml_):
    xml = ""
    with open('test.xml', 'w') as fw:
        print('(O-O)> {hello]')
        fw.writelines(xml_)

    return xml

def client_send_recv(port, file):
    s.sendto(file.encode('utf-8'), UDP_INFO)
    xml_encoded, addr = s.recvfrom(1024)
    print(xml_encoded.decode('utf-8'))

UDP_INFO = ("127.0.0.1", 4434)

print("UDP target IP: %s" % UDP_INFO[0])
print("UDP target port: %s" % UDP_INFO[1])

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ipoc = 123456123456

xml_ = []
xml_.append('<Rob TYPE="KUKA">\n')
xml_.append('<Gripper>1</Gripper>\n')
xml_.append('<StopFlag>0</StopFlag>\n')
xml_.append('<IPOC>')
xml_.append(str(ipoc))
xml_.append('</IPOC>\n')
xml_.append('</Rob>')

while True:
    s.sendto(xml_.encode('utf-8'), UDP_INFO)
    ipoc += 1
    time.sleep(1)


def xml_writer2(xml_):
    xml = ""
    with open('Send_msg_IPOCna.xml', 'w') as fw:
        print('(O-O)> {hello]')
        fw.writelines(xml_)

    return xml

xml_ = []
xml_.append('<?xml version="1.0" encoding="UTF-8"?>')
xml_.append('\n')
xml_.append('<Sen Type="ImFree">')
xml_.append('\n')
xml_.append('\t<Estr></Estr>')
xml_.append('\n')
xml_.append('\t<DiO></DiO>')
#xml_.append('\n')
#xml_.append('\t<IPOC></IPOC>')
xml_.append('\n')
xml_.append('</Sen>')
xml_writer2(xml_)