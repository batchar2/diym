#!/usr/bin/env python3

import os
import sys
import time
import socket
import threading

import numpy as np
import sounddevice as sd

import paho.mqtt.client as mqtt

UNIX_SOCKET = '/tmp/wavplayer.s'
SAMPLE_RATE = 9600#9600#11025
BLOCK_SIZE = 180


class SoundOut:
    recv_data = None

    def __init__(self):

        self.client = mqtt.Client()
        self.client.username_pw_set("d", "d")
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message

        self.client.connect("localhost", 1883, 60)
        self.client.subscribe("/dm/state/device_voice", qos=1)

        self.stream = sd.OutputStream(channels=2,
            dtype='int16',
            samplerate=SAMPLE_RATE,
            callback=self.callback,
            blocksize=BLOCK_SIZE)

    def on_subscribe(self, client, userdata, mid, granted_qos):
        pass
        #print("Subscribed: "+str(mid)+" "+str(granted_qos))

    def on_message(self, client, userdata, msg):
        """
        raw_data = msg.payload
        np_data = np.fromstring(raw_data, dtype=np.int16)
        gen_data = np.zeros((BLOCK_SIZE, 2))
        quant = (65536/256)
        index = 0
        for ch in np_data:
            value = round(ch*quant-32768)
            gen_data[index][0] = value
            gen_data[index][1] = value
            index += 1
        self.recv_data = gen_data
        """
        raw_data = msg.payload

        #quant = (256/65536)
        np_data = np.array([ch for ch in raw_data], dtype=np.int16)
        gen_data = np.zeros((BLOCK_SIZE, 2))

        quant = (65536/256)
        index = 0
        for ch in np_data:
            value = round(ch*quant-32768)
            gen_data[index][0] = value
            gen_data[index][1] = value
            index += 1
        self.recv_data = gen_data
        print(np_data)
        #print(self.recv_data)
        #print(len(raw_data), raw_data)
        print("----------------")

    def callback(self, outdata, frames, time, status):
        if self.recv_data is not None:
            #print(self.recv_data)
            outdata[:] = self.recv_data#q.get()#data#np.zeros((BLOCK_SIZE, 2))#data
            self.recv_data = None
        else:
            outdata[:] = np.zeros((BLOCK_SIZE, 2))

    def start(self):
        self.run()

    def run(self):
        with self.stream:
            def thread_mqtt():
                self.client.loop_forever()

            th = threading.Thread(target=thread_mqtt)
            th.start()

            while True:
                sd.sleep(int(10* 1000))

        #with self.stream:
        #while True:
        #    sd.sleep(int(10* 1000))




if __name__ == '__main__':
    out = SoundOut()
    out.start()
