import socket
MY_IP=str(socket.gethostbyname(socket.gethostname()))
#MY_IP= "10.42.125.139"
# this computers ip

IP_BLIMP_SUBNET='10.42.125.'
#blimp ip subnet
IP_BLIMP_INIT=50
#ip that blimps start at

ALIEN_PORT="/dev/ttyUSB0"
# port where the little frog lookin device is plugged in
