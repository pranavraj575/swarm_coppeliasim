import cv2
import io
import multiprocessing as mp
import numpy as np
import socket
import signal
import struct
import threading
import time
import traceback

from multiprocessing import shared_memory as sm
from numpy import pi

def handle_sensors(cfg, names, locks):
    """ private sensor handler spawned by a parent MultiRcv object
        - this handle should NOT be used by an application
        - only spawned and controlled by a parent MultiRcv object    
    """
    # instantiate threads to handle sensor stream
    rcv = _Rcv(cfg, names, locks)
    rcv.run()

class _Rcv:
    """ private sensor handler that runs on a spawned process to read data
        - this handler should NOT be used by an application
        - only spawned and controlled by a parent MultiRcv object    
    """
    
    def __init__(self, cfg, names, locks):        
        self.cfg = cfg

        # split the locks
        self.lock_img = locks[0]
        self.lock_bno = locks[1]
        self.lock_dis = locks[2]
        
        # setup the shared memory
        self.sh_img = sm.SharedMemory(names[0])
        self.sh_bno = sm.SharedMemory(names[2])
        self.sh_dis = sm.SharedMemory(names[4])
        self.sh_img_stamp = sm.SharedMemory(names[1])
        self.sh_bno_stamp = sm.SharedMemory(names[3])
        self.sh_dis_stamp = sm.SharedMemory(names[5])
        self.sh_flag = sm.SharedMemory(names[6])

        # interfaces for accessing shared memory
        im_sz = (cfg['im_rows'], cfg['im_cols'], 3)        
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.bno = np.ndarray(10, dtype=np.double, buffer=self.sh_bno.buf)
        self.dis = np.ndarray(1, dtype=np.double, buffer=self.sh_dis.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)
        self.bno_stamp = np.ndarray(1, np.double, buffer=self.sh_bno_stamp.buf)
        self.dis_stamp = np.ndarray(1, np.double, buffer=self.sh_dis_stamp.buf)
        self.running = np.ndarray(1, dtype=np.bool_, buffer=self.sh_flag.buf)

        # setup threads
        self.img_thread = threading.Thread(target=self.handle_img_read, args=())
        self.bno_thread = threading.Thread(target=self.handle_bno_read, args=())
        self.dis_thread = threading.Thread(target=self.handle_dis_read, args=())

        # setup control-c handler
        signal.signal(signal.SIGINT, self.handler)

    def run(self):
        # number of times to attempt connection
        num_tries = 10

        # prep the start signal
        my_ip = self.cfg['my_ip'].split('.')
        my_ip = [int(x) for x in my_ip]
        
        id_data = struct.pack("<1B", self.cfg['id_num'])
        ip_data = struct.pack("<4B", *my_ip)
        res_data = struct.pack("<2H", *(self.cfg['im_cols'], self.cfg['im_rows']))
        fps_data = struct.pack("<1B", self.cfg['fps'])
        qual_data = struct.pack("<1B", self.cfg['qual'])
        data = id_data + ip_data + res_data + fps_data + qual_data

        # send the start signal
        sock = socket.socket()
        sock.settimeout(0.5)
        connected = False
        while not connected and self.running[0]:
            try:
                sock.connect((self.cfg['pi_ip'], 8485))
                connected = True

            except: pass

        success = False
        if connected:
            con = sock.makefile('wb')
            con.write(data)
            try:
                con.flush()
                success = True
            except: pass

        sock.close()
        if success and connected: print("Connected to", self.cfg['pi_ip'])
        else: print("Couldn't connect to", self.cfg['pi_ip']); return
            
        # start the threads
        self.img_thread.start()
        self.bno_thread.start()
        self.dis_thread.start()

        # wait for threads to finish and then performs cleanup
        self.img_thread.join()
        self.bno_thread.join()
        self.dis_thread.join()

        # tell the pi to stop
        print("Putting blimp to sleep!")
        msg = "SLEEP000000"
        data = struct.pack("<11s", msg.encode('UTF-8'))

        sock = socket.socket()
        sock.settimeout(0.5)
        connected, tries = False, 0
        while not connected and tries < num_tries:
            try:
                sock.connect((self.cfg['pi_ip'], 8485))
                connected = True

            except: tries += 1        

        success = True
        if tries < num_tries:
            con = sock.makefile('wb')
            con.write(data)
            try: con.flush()
            except: success = False

        if success: print("Ending! Putting blimp", self.cfg['id_num'], "to sleep...")
        else: print("Connection to ", self.cfg['pi_ip'], " failed! Skipping sleep request...")
        con.close()

        
    def handler(self, signum, frame):
        # - ctrl-c handler to start process cleanup
        self.running[0] = False
        print("Ctrl-C shutdown of pi rcv!")

    def handle_img_read(self):
        # - parses all image data coming from the pi and stores it
        # - into shared memory. Uses protected writes to make sure
        # - race conditions are avoided

        # setup socket
        sock = socket.socket()
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
        sock.settimeout(1.0)
        sock.bind(('0.0.0.0', 1024+3*(self.cfg['id_num']-1)))
        sock.listen(0)
        
        connected = False
        while self.running[0] and not connected:
            try:
                img_connection = sock.accept()[0].makefile('rb')
                connected = True
            
            except: pass

        while self.running[0]:
            # read image size
            try: data = img_connection.read(struct.calcsize('<L'))
            except: data = None
 
            # parse the data if available
            if data is not None:
                # ignore if wrong size
                if len(data) != struct.calcsize('<L'): continue
                image_len = struct.unpack('<L', data)[0]

                # read image data
                image_stream = io.BytesIO()
                try: img_data = img_connection.read(image_len)
                except: traceback.print_exc()

                # ignore if wrong size
                if len(img_data) != image_len: continue
                image_stream.write(img_data)
                image_stream.seek(0)

                # parse image
                try:
                    nparr = np.frombuffer(image_stream.read(), np.uint8)
                    imgBGR = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                    img = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)

                    # read current time
                    tframe = np.frombuffer(img_connection.read(8), np.double)
                    img_stamp = tframe[0]

                    # protected update (wait for lock to be free)
                    is0, is1, is2 = img.shape
                    self.lock_img.acquire()
                    self.img[:is0,:is1,:is2] = img
                    self.img_stamp[0] = img_stamp       
                    self.lock_img.release()

                except: traceback.print_exc()

        sock.close()
        self.sh_img.close()
        self.sh_img_stamp.close()

    def handle_bno_read(self):
        # - parses all bno data coming from the pi and stores it
        # - into shared memory. Uses protected writes to make sure
        # - race conditions are avoided

        # setup socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
        sock.setblocking(0)
        sock.settimeout(0.5)        
        sock.bind(('0.0.0.0', 1025+3*(self.cfg['id_num']-1)))
        
        while self.running[0]:
            try: data, _ = sock.recvfrom(88)
            except: data = None

            # parse if available                
            if data is not None:
                # ignore if wrong size
                if len(data) != 88: continue
                
                # parse into bno data            
                new_bno = np.array(struct.unpack("<11d", data))

                # convert gyro to rads/sec
                new_bno[4:7] = new_bno[4:7]*pi/180.

                # protected update (skip if lock isn't available)
                if self.lock_bno.acquire(False):                    
                    self.bno[:10] = new_bno[:-1]
                    self.bno_stamp[0] = new_bno[-1]
                    self.lock_bno.release()
        
        sock.close()
        self.sh_bno.close()
        self.sh_bno_stamp.close()
        
    def handle_dis_read(self):
        # - parses all distance data coming from the pi and stores it
        # - into shared memory. Uses protected writes to make sure
        # - race conditions are avoided

        # setup socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(0)
        sock.settimeout(0.5)        
        sock.bind(('0.0.0.0', 1026+3*(self.cfg['id_num']-1)))

        while self.running[0]:
            try: data, _ = sock.recvfrom(16)
            except: data = None

            # parse if available
            if data is not None:
                # ignore if wrong size
                if len(data) != 16: continue

                # parse into distance data
                new_dis = struct.unpack("<2d", data)

                # protected update (skip if lock isn't available)
                if self.lock_dis.acquire(False):
                    self.dis[0] = new_dis[0]
                    self.dis_stamp[0] = new_dis[1]
                    self.lock_dis.release()

        sock.close()
        self.sh_dis.close()
        self.sh_dis_stamp.close()

