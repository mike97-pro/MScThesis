import cv2
import pickle
import pyaudio
import socket
import struct
import threading
import time
from datetime import datetime, timezone
import csv
from hapticData import HapticSerial
import servo_controller

keyboardLock = threading.Lock()
end = False


class VideoCall:
    global end

    def __init__(self, hostIP, destIP, LOG=False):
        self.h, self.w, self.Pframe = None, None, None
        self.hostIP = hostIP
        self.destIP = destIP
        self.ServerVideoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ClientVideoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ServerAudioSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ClientAudioSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.videoLock = threading.Lock()
        self.LOG = LOG


    def server_video(self, port=9999, qual=80, FPS = 30):
        global end
        vid = cv2.VideoCapture(0)

        self.ServerVideoSocket.bind((self.hostIP, port))
        print("Server video: Socket Bind Successfully")

        self.ServerVideoSocket.listen(5)
        print("Server video LISTENING AT:", (self.hostIP, port))

        try:
            while not end:
                client_socket, addr = self.ServerVideoSocket.accept()
                print('Server video GOT CONNECTION FROM:', addr)
                if client_socket:

                    start = time.time()
                    while vid.isOpened():
                        img, frame = vid.read()

                        if time.time() - start > 1./FPS:
                            start = time.time()

                            frame = cv2.flip(frame, 1)
                            ret, code = cv2.imencode('.JPEG', frame, [int(cv2.IMWRITE_JPEG_QUALITY), qual])
                            data = (code, datetime.now(timezone.utc))
                            a = pickle.dumps(data)
                            message = struct.pack("Q", len(a)) + a
                            client_socket.sendall(message)

                            if self.Pframe is not None:
                                cv2.rectangle(frame, (0, 0), (frame.shape[1], frame.shape[0]), (0, 255, 0), 3)
                                frame = cv2.resize(frame, (int(0.25 * self.w), int(0.25 * self.h)), cv2.INTER_AREA)
                                self.videoLock.acquire()
                                self.Pframe[5:5 + frame.shape[0], int(0.75 * self.w):int(0.75 * self.w) + frame.shape[1],
                                :] = frame
                                cv2.imshow("RECEIVING VIDEO", self.Pframe)
                                self.videoLock.release()

                        keyboardLock.acquire()
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q') or end:
                            client_socket.close()
                            vid.release()
                            end = True
                            keyboardLock.release()
                            break
                        keyboardLock.release()
            self.ServerVideoSocket.close()
        except:
            self.ServerVideoSocket.close()

    def server_audio(self, port=5000, chunk=6024, FORMAT=pyaudio.paInt16, CHANNELS=1, RATE=44100):
        global end
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=chunk)

        self.ServerAudioSocket.bind((self.hostIP, port))
        print("Server audio: Socket Bind Successfully")

        self.ServerAudioSocket.listen(5)
        print("Server audio LISTENING AT:", (self.hostIP, port))

        try:
            while not end:
                client_socket, addr = self.ServerAudioSocket.accept()
                print('Server audio GOT CONNECTION FROM:', addr)
                if client_socket:
                    while True:
                        audio = stream.read(chunk)
                        if audio:
                            data = (audio, datetime.now(timezone.utc))
                            a = pickle.dumps(data)
                            message = struct.pack("Q", len(a)) + a
                            client_socket.sendall(message)

                        keyboardLock.acquire()
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q') or end:
                            client_socket.close()
                            end = True
                            keyboardLock.release()
                            break
                        keyboardLock.release()

            self.ServerAudioSocket.close()
            stream.close()
        except:
            self.ServerAudioSocket.close()
            stream.close()

    def client_video(self, destPort):
        global end
        connected = False
        timeout = int(round(time.time()))
        while not connected:
            if int(round(time.time())) - timeout > 1:
                timeout = int(round(time.time()))
                try:
                    self.ClientVideoSocket.connect((self.destIP, destPort))
                    connected = True
                except:
                    connected = False

        data = b""
        payload_size = struct.calcsize("Q")
        print("Client Video: Socket Accepted")

        writer = None
        f = None
        if self.LOG:
            f = open('./client_video_log.csv', 'w')
            writer = csv.writer(f)
            writer.writerow(['Latency[s]'])

        try:
            while not end:
                while len(data) < payload_size:
                    packet = self.ClientVideoSocket.recv(2160)
                    if not packet: break
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]

                while len(data) < msg_size:
                    data += self.ClientVideoSocket.recv(2160)
                frame_data = data[:msg_size]
                data = data[msg_size:]
                code, timeStamp = pickle.loads(frame_data)

                self.videoLock.acquire()
                self.Pframe = cv2.imdecode(code, cv2.IMREAD_UNCHANGED)
                self.h, self.w = self.Pframe.shape[:2]
                self.videoLock.release()

                if self.LOG:
                    lat = datetime.now(timezone.utc)-timeStamp
                    writer.writerow([str(lat.total_seconds())])

                keyboardLock.acquire()
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or end:
                    end = True
                    keyboardLock.release()
                    break
                keyboardLock.release()

            self.ClientVideoSocket.close()
            if f:
                f.close()
        except:
            self.ClientVideoSocket.close()
            if f:
                f.close()

    def client_audio(self, destPort, chunk=1024, FORMAT=pyaudio.paInt16, CHANNELS=1, RATE=44100):
        global end
        time.sleep(1)
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        output=True,
                        frames_per_buffer=chunk)
        connected = False
        timeout = int(round(time.time()))
        while not connected:
            if int(round(time.time())) - timeout > 1:
                timeout = int(round(time.time()))
                try:
                    self.ClientAudioSocket.connect((self.destIP, destPort))
                    connected = True
                except:
                    connected = False

        data = b""
        payload_size = struct.calcsize("Q")
        print("Client audio: Socket Accepted")

        writer = None
        f = None
        if self.LOG:
            f = open('./client_audio_log.csv', 'w')
            writer = csv.writer(f)
            writer.writerow(['Latency[s]'])

        try:
            while not end:
                while len(data) < payload_size:
                    packet = self.ClientAudioSocket.recv(2160)
                    if not packet: break
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]

                while len(data) < msg_size:
                    data += self.ClientAudioSocket.recv(2160)
                audio_data = data[:msg_size]
                data = data[msg_size:]
                audio, timeStamp = pickle.loads(audio_data)
                stream.write(audio)

                if self.LOG:
                    lat = datetime.now(timezone.utc)-timeStamp
                    writer.writerow([str(lat.total_seconds())])

                keyboardLock.acquire()
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or end:
                    end = True
                    keyboardLock.release()
                    break
                keyboardLock.release()
            stream.close()
            self.ClientAudioSocket.close()
        except:
            stream.close()
            self.ClientAudioSocket.close()

    def start(self):
        x1 = threading.Thread(target=self.server_video)
        x2 = threading.Thread(target=self.server_audio)
        x3 = threading.Thread(target=self.client_video, args=[9999])
        x4 = threading.Thread(target=self.client_audio, args=[5000])

        x1.start()
        x2.start()
        x3.start()
        x4.start()

        x1.join()
        x2.join()
        x3.join()
        x4.join()


class HapticChannel:

    def __init__(self, hostIP, destIP, side='Patient', ttyStr='COM5', baud=9600, LOG=False):
        self.hostIP = hostIP
        self.destIP = destIP
        self.side = side
        self.thimble = HapticSerial(ttyStr, baud)
        self.LOG = LOG

    def server_haptic(self, port=6000):
        global end
        ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        ServerSocket.bind((self.hostIP, port))
        print("Server haptic: Socket Bind Successfully")

        ServerSocket.listen(5)
        print("Server haptic LISTENING AT:", (self.hostIP, port))

        try:
            while not end:
                client_socket, addr = ServerSocket.accept()
                print('Server haptic GOT CONNECTION FROM:', addr)
                if client_socket:
                    buff = []
                    while True:
                        meas = None
                        if len(buff) <= 100:
                            meas = self.thimble.readData()
                            if meas:
                                if len(meas) == 5:
                                    buff.append(meas)
                        else:
                            data = (buff, datetime.now(timezone.utc))
                            a = pickle.dumps(data)
                            message = struct.pack("Q", len(a)) + a
                            client_socket.sendall(message)
                            buff = []

                        keyboardLock.acquire()
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q') or end:
                            client_socket.close()
                            end = True
                            keyboardLock.release()
                            break
                        keyboardLock.release()
            ServerSocket.close()
        except:
            ServerSocket.close()

    def client_haptic(self, destPort, th0):
        global end

        self.thimble.writeAngle(th0)

        ClientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        timeout = int(round(time.time()))
        while not connected :
            if int(round(time.time())) - timeout > 1:
                timeout = int(round(time.time()))
                try:
                    ClientSocket.connect((self.destIP, destPort))
                    connected = True
                except:
                    connected = False


        data = b""
        payload_size = struct.calcsize("Q")
        print("Client haptic: Socket Accepted")
        writer = None
        f = None
        if self.LOG:
            f = open('./client_haptic_log.csv', 'w')
            writer = csv.writer(f)
            writer.writerow(['F1', 'F2', 'F3', 'F4', 'Time[ms]', 'Latency[s]'])

        try:
            while not end:
                while len(data) < payload_size:
                    packet = ClientSocket.recv(2160)
                    if not packet: break
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]

                while len(data) < msg_size:
                    data += ClientSocket.recv(2160)

                haptic_data = data[:msg_size]
                data = data[msg_size:]
                reading, timeStamp = pickle.loads(haptic_data)
                for k in reading:
                    self.thimble.writeForce(k, th0)

                if self.LOG:
                    lat = datetime.now(timezone.utc)-timeStamp
                    for k in reading:
                        row = [str(i) for i in k]
                        row.append(str(lat.total_seconds()))
                        writer.writerow(row)

                keyboardLock.acquire()
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or end:
                    end = True
                    keyboardLock.release()
                    break
                keyboardLock.release()

            ClientSocket.close()
            if f:
                f.close()
        except:
            ClientSocket.close()
            if f:
                f.close()
            print('haptic socket error')

    def activate(self, destPort, th0=178.55):

        if self.side == 'Doctor':
            self.client_haptic(destPort, th0)
        elif self.side == 'Patient':
            self.server_haptic()
        else:
            print("Haptic channel error: side not recognized")

        '''x3 = threading.Thread(target=self.server_haptic)
        x4 = threading.Thread(target=self.client_haptic, args=[destPort])

        x3.start()
        x4.start()
        x3.join()
        x4.join()'''
