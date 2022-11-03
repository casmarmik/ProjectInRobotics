#!/usr/bin/env python
# coding: utf-8

# In[1]:


import socket
from IPython.display import clear_output, display


# In[2]:


def find_free_port(s):
    s.bind(('localhost', 0))        # Bind to a free port provided by the host.
    return (s, s.getsockname()[1])  # Return the port number assigned.


# In[ ]:


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s, port = find_free_port(s_)
#print("Port: " + str(port))

HOST = "127.0.0.1"#"192.168.1.100"
PORT = 4434 #49152
UDP_INFO = (HOST, PORT)
s.bind(UDP_INFO)
#print(s.getsockname()[1])

while True:
    xml_encoded, addr = s.recvfrom(1024)
    xml_decoded = xml_encoded.decode('utf-8')
    clear_output(wait=True)
    print(xml_decoded)
s.shutdown
   


# https://stackoverflow.com/questions/2838244/get-open-tcp-port-in-python
# https://wiki.python.org/moin/UdpCommunication
# https://stackoverflow.com/questions/2623054/how-to-send-raw-xml-in-python
# https://stackoverflow.com/questions/51497714/xml-through-socket-in-python
# https://blog.stephencleary.com/2009/07/xml-over-tcpip.html
# https://stackoverflow.com/questions/42855972/printing-on-the-same-line-on-a-jupyter-notebook

# https://stackoverflow.com/questions/7585435/best-way-to-convert-string-to-bytes-in-python-3
