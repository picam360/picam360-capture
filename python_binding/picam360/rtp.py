import threading
import atexit
import numpy as np
from socket import socket, AF_INET, SOCK_DGRAM
import binascii
import struct

HOST = ''
ADDRESS = "127.0.0.1"

class Rtp():
    
    thread_run = False
    seq_id = 0
    recv_port = 0
    recv_callback = None

    def __init__(self, *args, **kwargs):
        atexit.register(self.stop)
    
    def send_packet(self, sock, port, payloadtype, data):
        length = 8+12+len(data)
        buff = bytearray(8)
        buff[0] = 0xFF
        buff[1] = 0xE1
        buff[2] = (length >> 8) & 0xff
        buff[3] = (length >> 0) & 0xff
        buff[4] = 0x72 # r
        buff[5] = 0x74 # t
        buff[6] = 0x70 # p
        buff[7] = 0x00 # \0
        pack = struct.pack('!BBHII', 0, payloadtype, self.seq_id, 0, 0)
        buff.extend(pack)
        buff.extend(data)
        #print(binascii.hexlify(buff))
        sock.sendto(buff, (ADDRESS, port))
        self.seq_id += 1

    def _capture_packets(self):
        print("recv thread started : %d" % self.recv_port)
        sock = socket(AF_INET, SOCK_DGRAM)
        sock.bind((HOST, self.recv_port))
            
        xmp = False
        xmp_pos = 0
        xmp_len = 0
        marker = 0
        pack = None
        
        while self.thread_run:
            buff = bytearray(65536)
            data_len, address = sock.recvfrom_into(buff)
            i = 0
            while i < data_len:
                if xmp:
                    if xmp_pos == 2:
                        xmp_len = buff[i] << 8 #network(big) endian
                        xmp_pos += 1
                    elif xmp_pos == 3:
                        xmp_len += buff[i] << 0 #network(big) endian
                        xmp_pos += 1
                    elif xmp_pos == 4:
                        if buff[i] == 0x72: # r
                            xmp_pos += 1
                        else:
                            xmp = False
                    elif xmp_pos == 5:
                        if buff[i] == 0x74: # t
                            xmp_pos += 1
                        else:
                            xmp = False;
                    elif xmp_pos == 6:
                        if buff[i] == 0x70: # p
                            xmp_pos += 1
                        else:
                            xmp = False
                    elif xmp_pos == 7: # rtp header
                        if buff[i] == 0x00: # \0
                            xmp_pos += 1
                        else:
                            xmp = False
                    else:
                        if xmp_pos == 8:
                            pack = bytearray(xmp_len - 8)
                        if i + (xmp_len - xmp_pos) <= data_len:
                            pack[xmp_pos - 8:] = buff[i:i + (xmp_len - xmp_pos)]
                            i += xmp_len - xmp_pos - 1
                            xmp_pos = xmp_len
                            
                            if self.recv_callback is not None:
                                self.recv_callback(pack)
                            
                            pack = None
                            xmp = False
                            #print("packet")
                        else:
                            rest_in_buff = data_len - i
                            pack[xmp_pos - 8:] = buff[i:]
                            i = data_len - 1
                            xmp_pos += rest_in_buff
                            print("split")
                else:
                    if marker:
                        marker = 0
                        if buff[i] == 0xE1: #xmp
                            xmp = True
                            xmp_pos = 2
                            xmp_len = 0
                    elif buff[i] == 0xFF:
                        marker = 1
                i += 1
                        
        print("recv thread finished : %d" % self.recv_port)
    
    def start(self, port, callback):
        self.recv_port = port
        self.recv_callback = callback
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread_run = True
            self.thread = threading.Thread(target=self._capture_packets)
            self.thread.start()

    def stop(self):
        if hasattr(self, 'thread'):
            self.thread_run = False
            self.thread.join()
            
    def restart(self):
        self.stop()
        self.start()