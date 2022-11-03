#!/usr/bin/env python
# coding: utf-8

# In[10]:


import socket


# In[11]:


def find_free_port(s):
    s.bind(('localhost', 0))        # Bind to a free port provided by the host.
    return (s, s.getsockname()[1])  # Return the port number assigned.


# In[14]:


s_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s, port = find_free_port(s_)
print("Port: " + str(port))

xml_decoded = ""
while xml_decoded == "" :
    xml_encoded, addr = s.recvfrom(1024) # buffer size is 1024 bytes
    xml_decoded = xml_encoded.decode('utf-8')
    print(xml_decoded)
s.shutdown



# In[ ]:





# https://stackoverflow.com/questions/2838244/get-open-tcp-port-in-python
# https://wiki.python.org/moin/UdpCommunication
# https://stackoverflow.com/questions/2623054/how-to-send-raw-xml-in-python
# https://stackoverflow.com/questions/51497714/xml-through-socket-in-python
# https://blog.stephencleary.com/2009/07/xml-over-tcpip.html

# https://stackoverflow.com/questions/7585435/best-way-to-convert-string-to-bytes-in-python-3
