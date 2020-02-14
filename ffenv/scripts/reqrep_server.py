import zmq
import time
import sys

port = "5556"

if len(sys.argv) > 1:
    port = sys.argv[1]
    int(port)

# Creating a context
context = zmq.Context()
# Creating a socket
socket = context.socket(zmq.REP)
# Connecting/ Binding a socket to port
socket.bind("tcp://*:%s" % port)

while True:
    # Wait for next request from client
    message = socket.recv()
    print ("Recieved request : ", message)
    time.sleep(0.1)
    # Sending a message to client
    socket.send_string("World from %s" % port)