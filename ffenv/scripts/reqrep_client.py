import zmq
import sys

port = "5556"

if len(sys.argv) > 1:
    port = sys.argv[1]
    int(port)

# Creating a context
context = zmq.Context()
print ("Connecting to server...")
# Creating a socket
socket = context.socket(zmq.REQ)
# Connecting socket to port
socket.connect("tcp://localhost:%s" % port)

# Do 10 requests, waiting each time for a response
for request in range(1, 10):
    print ("Sending request ", request, "...")
    # Sending message to server
    socket.send_string ("Hello")
    # Waiting to recieve reply from server
    message = socket.recv()
    print ("Recieved reply ", request, "[", message, "]")