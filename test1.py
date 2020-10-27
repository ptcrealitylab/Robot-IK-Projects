# Echo client program
import socket
import time
HOST = "192.168.1.5" # The remote host
PORT = 30002 # The same port as used by the server
print("Starting Program")
count = 0
while (count < 1):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((HOST, PORT))
	time.sleep(0.5)
	print("Set output 1 and 2 high")
	s.send (b"set_digital_out(1,True)" + b"\n")
	time.sleep(0.1)

	s.send (b"set_digital_out(2,True)" + b"\n")
	time.sleep(2)
	print("Robot starts Moving to 3 positions based on joint positions")
	s.send (b"movej([-4, -4, -4, -4, -4, -4], a=1.0, v=0.1)" + b"\n")
	time.sleep(10)
	print("Set output 1 and 2 low")
	s.send (b"set_digital_out(1,False)" + b"\n")
	time.sleep(0.1)

	s.send (b"set_digital_out(2,False)" + b"\n")
	time.sleep(0.1)
	count = count + 1
	print("The count is:", count)
	print("Program finish")
	time.sleep(1)
	data = s.recv(1024)
	s.close()
	print(("Received", repr(data)))
print("Status data received from robot")