#
#   Connects SUB socket to tcp://localhost:7776
#

import sys
import zmq

#  Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collecting sensor data")
socket.connect("tcp://localhost:7776")

# Subscribe to everything
data_filter = ''
if isinstance(data_filter, bytes):
    data_filter = data_filter.decode('ascii')
socket.setsockopt_string(zmq.SUBSCRIBE, data_filter)


while True: 
	string = socket.recv_json()
	print string

