#!/usr/bin/env python
# coding: utf-8

# ## Client
# Links: 
# https://stackoverflow.com/questions/2623054/how-to-send-raw-xml-in-python
# https://wiki.python.org/moin/UdpCommunication

# https://www.pythontutorial.net/python-basics/python-read-text-file/

# In[1]:


import socket
import time


# In[2]:


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


# In[3]:


xml_ = []
xml_.append('<xml version="1.0" encoding="UTF-8">')
xml_.append('\n')
xml_.append('<header/>')
xml_.append('\n')
xml_.append('<body>')
xml_.append('\n')
xml_.append('\t<code>')
xml_.append('\n')
xml_.append('<body/>')

xml = xml_sum(xml_)

xml_writer(xml_)

file_ = ""
with open('test.xml', 'r') as fr:
    for line in fr.readlines():
        file_ += line.strip()
        file_ += '\n'
print(file_)


# In[4]:


def client_send(port, file):


    #xml_encoded = file.encode('utf-8')

    UDP_INFO = ("127.0.0.1", port)

    print("UDP target IP: %s" % UDP_INFO[0])
    print("UDP target port: %s" % UDP_INFO[1])
    print("xml:\n %s" % file)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(file.encode('utf-8'), UDP_INFO)


# In[6]:


client_send(4434, file_)


# In[ ]:




