#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#https://docs.python.org/3/library/xml.etree.elementtree.html
#https://stackoverflow.com/questions/70926664/how-can-i-find-an-element-by-name-in-a-xml-file
#https://docs.python.org/3/library/xml.etree.elementtree.html#xml.etree.ElementTree.Element.text
#https://stackoverflow.com/questions/23001757/python-elementtree-no-element-found-exception

import socket
import xml.etree.ElementTree as et

def xml_writer(xml_):
    xml = ""
    with open('extraction_file.xml', 'w') as fw:
        fw.writelines(xml_)
        print('(O-O)> {Extraction_file_creation_complete]')

    return xml

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
HOST = "127.0.0.1"#"192.168.1.100"
PORT = 4434 #49152
UDP_INFO = (HOST, PORT)
s.bind(UDP_INFO)

while True:
    xml_encoded, addr = s.recvfrom(1024)
    xml_decoded = xml_encoded.decode('utf-8')
    print(xml_decoded)
    root = et.fromstring(str(xml_decoded))
    ipoc = root.find('IPOC').text
    print('(O-O)> {IPOC: ' + ipoc +']\n')
    file_ = ""
    with open('Send_msg_IPOCna.xml', 'r') as fr:
        for line in fr.readlines():
            file_ += line.strip()
            file_ += '\n'
    print(file_)
    root = et.fromstring(file_)
    ipoc_element = et.Element("IPOC")
    ipoc = int(ipoc) + 1
    ipoc = str(ipoc)
    ipoc_element.text = ipoc

    l = list(root.iter())
    root.insert(len(l) - 1, ipoc_element)
    xml_reply = et.tostring(root, encoding='utf-8', method='xml')
    print(xml_reply.decode('utf-8'))
    
    s.sendto(xml_reply, UDP_INFO)

s.shutdown
   


# In[ ]:





# In[ ]:




