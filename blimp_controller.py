import curses
import time
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# Initialize the terminal
win = curses.initscr()

# Turn off line buffering
curses.cbreak()


# Make getch() non-blocking
win.nodelay(True)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
while True:
    key = win.getch()
    if key != -1:
       print('Pressed key', key)
       # sock.sendto('Hello',(UDP_IP, UDP_PORT))
       print(type(key))
       sock.sendto(str.encode(str(key)), (UDP_IP, UDP_PORT))
    time.sleep(0.01)

