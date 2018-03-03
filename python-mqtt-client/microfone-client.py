#!/usr/bin/env python3

import os
import sys

import numpy as np
import sounddevice as sd

import paho.mqtt.client as mqtt


SAMPLE_RATE = 9600
BLOCK_SIZE = 160



class Microphone:
    def __init__(self):

        self.client = mqtt.Client()
        self.client.username_pw_set("d", "d")
        self.client.connect("localhost", 1883, 60)

        self.stream = sd.InputStream(channels=2,
            dtype='int16',
            samplerate=SAMPLE_RATE,
            callback=self.callback,
            blocksize=BLOCK_SIZE)

    def callback(self, indata, frames, time, status):
        quant = (256/65536)
        data_array = [int(((ch[0])+32768)*quant) for ch in indata]
        np_data = np.array(data_array, dtype=np.int16)
        #print(np_data)
        #self.sock.sendto(np_data.tobytes(), self.server_address)

        self.client.publish("/dm/test", np_data.tobytes());
        #sys.exit()
    def start(self):
        with self.stream:
            while True:
                sd.sleep(int(10*1000))


m = Microphone()
m.start()

#client.disconnect();
