#!/usr/bin/env/python3
import multiprocessing as mp
import numpy as np
import signal
import socket
import struct
import time
import traceback

from multiprocessing import shared_memory as sm


class MultiStream:
    def __init__(self):
        # setup the process names
        self.pcam = None
        self.pbno = None
        self.pdis = None

        # setup shared memory
        self.sh_t0 = sm.SharedMemory(create=True, size=24)
        self.sh_ip = sm.SharedMemory(create=True, size=4)
        self.sh_c_port = sm.SharedMemory(create=True, size=2)
        self.sh_b_port = sm.SharedMemory(create=True, size=2)
        self.sh_d_port = sm.SharedMemory(create=True, size=2)
        self.sh_params = sm.SharedMemory(create=True, size=8)

        self.t0 = np.ndarray(1, dtype=np.float64, buffer=self.sh_t0.buf)
        self.ip = np.ndarray(4, dtype=np.uint8, buffer=self.sh_ip.buf)
        self.c_port = np.ndarray(1, dtype=np.uint16, buffer=self.sh_c_port.buf)
        self.b_port = np.ndarray(1, dtype=np.uint16, buffer=self.sh_b_port.buf)
        self.d_port = np.ndarray(1, dtype=np.uint16, buffer=self.sh_d_port.buf)
        self.params = np.ndarray(4, dtype=np.uint16, buffer=self.sh_params.buf)

        self.t_lock = mp.Lock()
        self.ip_lock = mp.Lock()
        self.c_lock = mp.Lock()
        self.b_lock = mp.Lock()
        self.d_lock = mp.Lock()
        self.p_lock = mp.Lock()

        # setup the sleep/running flag
        self.sh_flag = sm.SharedMemory(create=True, size=2)
        self.flag = np.ndarray(2, dtype=np.bool_, buffer=self.sh_flag.buf)
        self.flag[:] = [False, True]  # active, running

        # running flag (to be stopped in ctrl-c event)
        self.running = True

        signal.signal(signal.SIGINT, self.handler)

        # get SM names
        t0_name = self.sh_t0.name
        ip_name = self.sh_ip.name
        cp_name = self.sh_c_port.name
        bp_name = self.sh_b_port.name
        dp_name = self.sh_d_port.name
        pa_name = self.sh_params.name
        fl_name = self.sh_flag.name

        # setup the processes
        c_names = (t0_name, ip_name, cp_name, pa_name, fl_name)
        c_locks = (self.t_lock, self.ip_lock, self.c_lock, self.p_lock)
        self.pcam = mp.Process(target=self.handle_cam, args=(c_names, c_locks))

        b_names = (t0_name, ip_name, bp_name, fl_name)
        b_locks = (self.t_lock, self.ip_lock, self.c_lock)
        self.pbno = mp.Process(target=self.handle_bno, args=(b_names, b_locks))

        d_names = (t0_name, ip_name, dp_name, fl_name)
        d_locks = (self.t_lock, self.ip_lock, self.c_lock)
        self.pdis = mp.Process(target=self.handle_dis, args=(d_names, d_locks))

    def handler(self, signum, frame):
        self.running = False

    def run(self):
        # setup the socket for listening
        sock = socket.socket()
        sock.bind(('0.0.0.0', 8485))
        sock.settimeout(0.5)
        sock.listen(0)

        # start the processes
        self.pcam.start()
        self.pbno.start()
        self.pdis.start()

        while self.running:
            # wait for a request
            connected = False
            while not connected and self.running:
                try:
                    con = sock.accept()[0].makefile('rb')
                    connected = True

                except: pass

            data = []
            if connected:
                try: data = con.read(11)
                except: pass

            # if stop signal is read, go to sleep
            if len(data) == 11:
                # parse data
                msg = "".join(map(chr, data))

                # check if correct message
                if msg == 'SLEEP000000':
                    self.flag[0] = False
                    print("Sleeping!")

                else:
                    # - 1 byte id_num, 4 bytes IP, 4 bytes img size, 1 byte fps, 1 byte quality

                    # protected writes
                    ip = struct.unpack("<4B", data[1:5])
                    self.ip_lock.acquire()
                    self.ip[:] = ip
                    self.ip_lock.release()

                    id_num = data[0]
                    self.c_lock.acquire()
                    self.c_port[:] = 1024 + 3*(id_num-1)
                    self.c_lock.release()
                    self.b_lock.acquire()
                    self.b_port[:] = 1025 + 3*(id_num-1)
                    self.b_lock.release()
                    self.d_lock.acquire()
                    self.d_port[:] = 1026 + 3*(id_num-1)
                    self.d_lock.release()

                    self.p_lock.acquire()
                    self.params[:2] = struct.unpack("<2H", data[5:9])
                    self.params[2] = data[9]
                    self.params[3] = data[10]
                    self.p_lock.release()

                    # (re)start streams
                    self.flag[0] = False
                    time.sleep(1)
                    self.t_lock.acquire()
                    self.t0[0] = time.time()
                    self.t_lock.release()
                    print(self.t0[0])
                    self.flag[0] = True

                    print("Connected to", ".".join(map(str, ip)))

        # shutdown operations
        print("Shutting down!")
        sock.close()
        self.flag[:] = False
        if self.pcam is not None:
            self.pcam.join()

        if self.pbno is not None:
            self.pbno.join()

        if self.pdis is not None:
            self.pdis.join()

        self.sh_t0.close()
        self.sh_ip.close()
        self.sh_params.close()
        self.sh_c_port.close()
        self.sh_b_port.close()
        self.sh_d_port.close()
        self.sh_flag.close()

        self.sh_t0.unlink()
        self.sh_ip.unlink()
        self.sh_params.unlink()
        self.sh_c_port.unlink()
        self.sh_b_port.unlink()
        self.sh_d_port.unlink()
        self.sh_flag.unlink()

    def handle_cam(self, names, locks):
        # see if camera libs are installed
        try:
            from picamera import PiCamera
            from io import BytesIO

        except:
            print("Camera library import failed! Ignoring camera stream")
            return

        # setup shared memory
        sh_t0 = sm.SharedMemory(names[0])
        sh_ip = sm.SharedMemory(names[1])
        sh_port = sm.SharedMemory(names[2])
        sh_params = sm.SharedMemory(names[3])
        sh_flag = sm.SharedMemory(names[4])

        t0 = np.ndarray(1, dtype=np.float64, buffer=sh_t0.buf)
        ip = np.ndarray(4, dtype=np.uint8, buffer=sh_ip.buf)
        port = np.ndarray(1, dtype=np.uint16, buffer=sh_port.buf)
        params = np.ndarray(4, dtype=np.uint16, buffer=sh_params.buf)
        flag = np.ndarray(2, dtype=np.bool_, buffer=sh_flag.buf)

        # shutdown flag
        while flag[1]:
            locks[1].acquire()
            ip_ = ip[:]
            locks[1].release()
            ip_ = ".".join(map(str, ip_))

            locks[2].acquire()
            port_ = port[:]
            locks[2].release()

            # connect to the server
            sock = socket.socket()
            sock.settimeout(2)

            connected = False
            while not connected and flag[0]:
                try:
                    sock.connect((ip_, port_[0]))
                    connected = True

                except: print("Waiting for camera server...")

            # return prematurely if stopped early
            if not connected:
                time.sleep(1)
                continue

            # setup the camera
            locks[3].acquire()
            params_ = params[:]
            locks[3].release()

            res, fps, q = params_[:2], params_[2], params_[3]

            cam = PiCamera()
            cam.resolution = res
            cam.framerate = fps
            cam.exposure_mode = "sports"

            print("Camera successfully registered")
            cam_connection = sock.makefile('wb')

            # start the camera stream
            locks[0].acquire()
            t0_ = t0[0]
            locks[0].release()
            buf = BytesIO()
            for _ in cam.capture_continuous(buf, format="jpeg", quality=q, use_video_port=True):
                # break if run flag is off
                if not flag[0]: break

                # get the current time
                tframe = time.time() - t0_

                try:
                    # write the length of the capture
                    buf_len = struct.pack('<L', buf.tell())
                    cam_connection.write(buf_len)
                    cam_connection.flush()

                    # write the image data
                    buf.seek(0)
                    cam_connection.write(buf.read())
                    cam_connection.flush()

                    # write the current time
                    tbuf = struct.pack('<d', tframe)
                    cam_connection.write(tbuf)
                    cam_connection.flush()

                    # delete buffer
                    buf.seek(0)
                    buf.truncate()

                except:
                    traceback.print_exc()

            # prep for next call
            cam.close()
            sock.close()
            time.sleep(1)

        # cleanup
        sock.close()
        sh_t0.close()
        sh_ip.close()
        sh_port.close()
        sh_params.close()
        sh_flag.close()

    def handle_bno(self, names, locks):
        # see if BNO055 libs are installed
        try:
            from Adafruit_BNO055 import BNO055

        except:
            print("BNO055 library import failed! Ignoring imu stream")
            return

        # setup the bno055
        sensor = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
        sensor.set_mode(BNO055.OPERATION_MODE_IMUPLUS)

        # setup shared memory
        sh_t0 = sm.SharedMemory(names[0])
        sh_ip = sm.SharedMemory(names[1])
        sh_port = sm.SharedMemory(names[2])
        sh_flag = sm.SharedMemory(names[3])

        t0 = np.ndarray(1, dtype=np.float64, buffer=sh_t0.buf)
        ip = np.ndarray(4, dtype=np.uint8, buffer=sh_ip.buf)
        port = np.ndarray(1, dtype=np.uint16, buffer=sh_port.buf)
        flag = np.ndarray(2, dtype=np.bool_, buffer=sh_flag.buf)

        # setup the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # shutdown flag
        last_flag = flag[0]
        while flag[1]:
            locks[0].acquire()
            t0_ = t0[0]
            locks[0].release()
            print(t0_)
            locks[1].acquire()
            ip_ = ip[:]
            locks[1].release()
            ip_ = ".".join(map(str, ip_))

            locks[2].acquire()
            port_ = port[:]
            locks[2].release()

            # active flag
            while flag[0]:
                if last_flag != flag[0]:
                    print("BNO055 successfully registered")

                quat = sensor.read_quaternion()
                gyro = sensor.read_gyroscope()
                acc  = sensor.read_linear_acceleration()
                tframe = time.time() - t0_
                print(tframe)
                bno_packet = quat + gyro + acc + (tframe,)

                bno_bytes = struct.pack("<11d", *bno_packet)
                sock.sendto(bno_bytes, (ip_, port_[0]))

                last_flag = flag[0]

            last_flag = flag[0]
            time.sleep(1)

        # cleanup
        sock.close()
        sh_t0.close()
        sh_ip.close()
        sh_port.close()
        sh_flag.close()


    def handle_dis(self, names, locks):
        # see if VL53L1X libs are installed
        try:
            import board
            import adafruit_vl53l1x

        except:
            print("VL53L1x library import failed! Ignoring distance stream")
            return

        # setup shared memory
        sh_t0 = sm.SharedMemory(names[0])
        sh_ip = sm.SharedMemory(names[1])
        sh_port = sm.SharedMemory(names[2])
        sh_flag = sm.SharedMemory(names[3])

        t0 = np.ndarray(1, dtype=np.float64, buffer=sh_t0.buf)
        ip = np.ndarray(4, dtype=np.uint8, buffer=sh_ip.buf)
        port = np.ndarray(1, dtype=np.uint16, buffer=sh_port.buf)
        flag = np.ndarray(2, dtype=np.bool_, buffer=sh_flag.buf)

        # setup the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # setup the VL53L1X
        i2c = board.I2C()
        vl53 = adafruit_vl53l1x.VL53L1X(i2c)
        vl53.distance_mode = 2
        vl53.timing_budget = 100

        # shutdown flag
        last_flag = flag[0]
        while flag[1]:
            locks[0].acquire()
            t0_ = t0[0]
            locks[0].release()

            locks[1].acquire()
            ip_ = ip[:]
            locks[1].release()
            ip_ = ".".join(map(str, ip_))

            locks[2].acquire()
            port_ = port[:]
            locks[2].release()

            # active flag
            while flag[0]:
                if last_flag != flag[0]:
                    vl53.start_ranging()
                    print("VL53L1X successfully registered")

                if vl53.data_ready:
                    distance = vl53.distance
                    vl53.clear_interrupt()

                    if distance is not None:
                        tframe = time.time() - t0_
                        dist_packet = (distance, tframe)
                        dist_bytes = struct.pack("<2d", *dist_packet)
                        sock.sendto(dist_bytes, (ip_, port_[0]))

                time.sleep(0.02)
                last_flag = flag[0]

            # low-power mode
            if last_flag != flag[0]: vl53.stop_ranging()
            last_flag = flag[0]
            time.sleep(1)

        # cleanup
        sock.close()
        sh_t0.close()
        sh_ip.close()
        sh_port.close()
        sh_flag.close()


# setup streamer
if __name__ == "__main__":
    S = MultiStream()
    S.run()
