import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
import threading
import numpy as np
from socket import socket, AF_INET, SOCK_DGRAM
import binascii
import struct
import re
import time
from .rtp import Rtp
import binascii

PT_STATUS = 100
PT_CMD = 101
PT_CAM_BASE = 110
STATUS_PORT = 9004
SEND_PORT = 9005
IMAGE_PORT = 9200

class Camera(SingletonConfigurable):
    
    thread_run = False
    last_frame_id = 0
    cmd_id = 0
    vos_id = 0;
    active_frame = None
    value = traitlets.Any()
    
    send_sock = None
    rtp = Rtp()
    
    # config
    width = traitlets.Integer(default_value=512).tag(config=True)
    height = traitlets.Integer(default_value=512).tag(config=True)

    def __init__(self, *args, **kwargs):
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        super(Camera, self).__init__(*args, **kwargs)
        
        self.send_sock = socket(AF_INET, SOCK_DGRAM)
        self._send_comand(self._delete_vostream_cmd())
        self._send_comand(self._create_vostream_cmd())
        self.start()

        atexit.register(self.stop)

    def set_vostream_param(self, value):
        if self.vos_id == 0:
            return
        cmd = "set_vostream_param id=%d %s" % (self.vos_id, value)
        self._send_comand(cmd)
        
    def _send_comand(self, _cmd):
        cmd = '<picam360:command id="%d" value="%s" />'
        cmd = cmd % (self.cmd_id, _cmd)
        #print(cmd)
        self.rtp.send_packet(self.send_sock, SEND_PORT, PT_CMD, cmd.encode())
        self.cmd_id += 1
        
    def _parse_status_value(self, name, value):
        pass
            
    def _parse_status(self, data):
        split = data.decode().split(' ')
        i = 0
        while i < len(split):
            if split[i].startswith('name='):
                name = re.split('[=,\"]', split[i])
                value = re.split('[=,\"]', split[i+1])
                self._parse_status_value(name[2], value[2])
                i += 1
            i += 1
        
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
                map = {}
                split = self.active_frame['meta'].split(' ')
                for i in range(len(split)):
                    _split = re.split('[=,\"]', split[i])
                    map[_split[0]] = _split
                self.vos_id = int(map['frame_id'][2])
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
                if self.active_frame['img_type'][2] == "RGBA":
                	self.value = np.frombuffer(self.active_frame['pixels'][0:w*h*4], dtype=np.uint8).reshape(h, w, 4)
                else:
                	self.value = np.frombuffer(self.active_frame['pixels'][0:w*h], dtype=np.uint8).reshape(h, w, 1)
                self.active_frame = None

    def _parse_packet(self, pack):
        b1, payloadtype, seq_id, i1, i2 = struct.unpack('!BBHII', pack[0:12])
        payloadtype = payloadtype & 0x7F
        if payloadtype == PT_STATUS:
            self._parse_status(pack[12:])
        if payloadtype == PT_CAM_BASE:
            self._parse_image(pack[12:])
                
    def _create_vostream_cmd(self):
        cmd = 'create_vostream WINDOW img_type=RGBA width=%d height=%d|%s|rtp port=%d'
        return cmd % (self.width, self.height, "dmem_tegra_converter", IMAGE_PORT)
                
    def _delete_vostream_cmd(self):
        cmd = "delete_vostream -i *"
        return cmd
    
    def start(self):
        self.rtp.start(IMAGE_PORT, self._parse_packet)

    def stop(self):
        self.rtp.stop()
            
    def restart(self):
        self.stop()
        self.start()