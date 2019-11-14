import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
import threading
import numpy as np
from socket import socket, AF_INET, SOCK_DGRAM
import time
import binascii
import struct
import re

HOST = ''
SEND_PORT = 9005
RECV_PORT = 9200
ADDRESS = "127.0.0.1"
PT_STATUS = 100
PT_CMD = 101
PT_CAM_BASE = 110

class Camera(SingletonConfigurable):
    
    thread_run = False
    cmd_id = 0
    seq_id = 0
    active_frame = None
    value = traitlets.Any()
    
    # config
    width = traitlets.Integer(default_value=512).tag(config=True)
    height = traitlets.Integer(default_value=512).tag(config=True)

    def __init__(self, *args, **kwargs):
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        super(Camera, self).__init__(*args, **kwargs)

        try:
            self.send_sock = socket(AF_INET, SOCK_DGRAM)
            self.recv_sock = socket(AF_INET, SOCK_DGRAM)
            self.recv_sock .bind((HOST, RECV_PORT))
        
            self._send_comand(self._delete_vostream_cmd())
            self._send_comand(self._create_vostream_cmd())

            self.value = None
            self.start()
        except:
            self.stop()
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')

        atexit.register(self.stop)

    def _send_comand(self, _cmd):
        cmd = '<picam360:command id="%d" value="%s" />'
        cmd = cmd % (self.cmd_id, _cmd)
        #print(cmd)
        self._send_packet(cmd.encode())
        self.cmd_id += 1
    
    def _send_packet(self, data):
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
        pack = struct.pack('!BBHII', 0, PT_CMD, self.seq_id, 0, 0)
        buff.extend(pack)
        buff.extend(data)
        #print(binascii.hexlify(buff))
        self.send_sock.sendto(buff, (ADDRESS, SEND_PORT))
        self.seq_id += 1
        
    def _parse_image(self, data):
        if self.active_frame == None:
            if data[0] == 0x50 and data[1] == 0x49: # 'P' 'I'
                length = (data[2] << 8) + (data[3] << 0)
                pif_header = data[4:4+length].decode()
                map = {}
                split = pif_header.split(' ')
                for i in range(len(split)):
                    _split = re.split('[=,\"]', split[i])
                    map[_split[0]] = _split
                self.active_frame = map
                width = self.active_frame['width'][2:5]
                stride = self.active_frame['stride'][2:5]
                height = self.active_frame['height'][2:5]
                image_size = 0
                for i in range(3):
                    image_size += int(stride[i]) * int(height[i])
                self.active_frame['image_size'] = image_size
                self.active_frame['pixels'] = bytearray(image_size)
                
                data = data[4+length:]
                if len(data) == 0:
                    return
            else:
                return
        if self.active_frame != None and 'meta' not in self.active_frame:
            meta_size = int(self.active_frame['meta_size'][2])
            if meta_size == 0:
                self.active_frame['meta'] = ""
            else:
                self.active_frame['meta'] = data[0:meta_size].decode()
            self.active_frame['pixels_cur'] = 0
            
            data = data[meta_size:]
            if len(data) == 0:
                return
        if self.active_frame != None:
            if self.active_frame['pixels_cur'] + len(data) > self.active_frame['image_size']:
                print("something wrong!")
                self.active_frame = None
                return
            self.active_frame['pixels'][self.active_frame['pixels_cur']:] = data[:]
            self.active_frame['pixels_cur'] += len(data)
            if self.active_frame['pixels_cur'] == self.active_frame['image_size']:
                #print("new image!")
                w = int(self.active_frame['width'][2])
                h = int(self.active_frame['height'][2])
                self.value = np.frombuffer(self.active_frame['pixels'][0:w*h], dtype=np.uint8).reshape(h, w, 1)
                self.active_frame = None

    def _parse_packet(self, pack):
        b1, payloadtype, seq_id, i1, i2 = struct.unpack('!BBHII', pack[0:12])
        if payloadtype == PT_CAM_BASE:
            self._parse_image(pack[12:])

    def _capture_frames(self):
        print("recv thread started")
        xmp = False
        xmp_pos = 0
        xmp_len = 0
        marker = 0
        pack = None
        
        while self.thread_run:
            buff = bytearray(65536)
            data_len, address = self.recv_sock.recvfrom_into(buff)
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
                            
                            self._parse_packet(pack)
                            
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
                        
        print("recv thread finished")
                
    def _create_vostream_cmd(self):
        cmd = 'create_vostream PICAM360MAP width=%d height=%d|%s|rtp port=%d'
        return cmd % (self.width, self.height, "dmem_tegra_converter", RECV_PORT)
                
    def _delete_vostream_cmd(self):
        cmd = "delete_vostream -i *"
        return cmd
    
    def start(self):
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread_run = True
            self.thread = threading.Thread(target=self._capture_frames)
            self.thread.start()

    def stop(self):
        if hasattr(self, 'thread'):
            self.thread_run = False
            self.thread.join()
            
    def restart(self):
        self.stop()
        self.start()