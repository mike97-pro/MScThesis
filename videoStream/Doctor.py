import consultation
import threading

if __name__ == '__main__':
    host_ip = '192.168.0.108'
    dest_ip = '192.168.0.103'

    call = consultation.VideoCall(host_ip, dest_ip, LOG=True)
    hChannel = consultation.HapticChannel(host_ip,dest_ip, side='Doctor', LOG=True, ttyStr='COM10')

    x1 = threading.Thread(target=hChannel.activate, args=[6000])
    x1.start()

    call.start()
    x1.join()