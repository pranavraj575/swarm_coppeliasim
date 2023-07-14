import socket
import os, sys

MY_IP=str(socket.gethostbyname(socket.gethostname()))
#MY_IP= "10.42.125.139"
# this computers ip

IP_BLIMP_SUBNET='10.42.125.'
#blimp ip subnet
IP_BLIMP_INIT=50
#ip that blimps start at

ALIEN_PORT="/dev/ttyUSB0"
# port where the little frog lookin device is plugged in

#should be .../physical_blimps
BLIMP_DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))

#should be .../physical_blimps/configs
CONFIGS_DIR = os.path.join(BLIMP_DIR, 'configs')

def create_config_file(blimp_id):
    """
    creates a config file for a given blimp
    @param blimp_id: id of blimp, starts from 0
    @return file name
    """
    path = os.path.join(CONFIGS_DIR, 'config' + str(blimp_id) + '.yaml')
    f = open(path, 'w')
    f.write('id_num: ' + str(blimp_id) + '\n')
    f.write('my_ip: "' + MY_IP + '"\n')
    f.write('pi_ip: "' + IP_BLIMP_SUBNET + str(blimp_id + IP_BLIMP_INIT) + '"\n')
    for line in (
            'im_rows: 240',
            'im_cols: 360',
            'fps: 15',
            'qual: 15',
            '# motor board parameters',
            'positive_only: False',
            'k_r:  [0.45, 0., -1.7]',
            'k_p:  [0.45, 0., -1.7]',
            'k_yw: [0.002, 0.001, 0.07]',
            'k_pz: [0.35, 0.001, 0.055]',
    ):
        f.write(line)
        f.write('\n')
    f.close()
    return path