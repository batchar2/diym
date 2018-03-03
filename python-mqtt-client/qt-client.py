#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import QThread

import paho.mqtt.client as mqtt


MQTT_CONTROL_PUB = '/dm/state/control'
MQTT_DEVICE_SUB  = '/dm/state/device'

class ThreadMQTT(QThread):

    def __init__(self, parent_win):
        QThread.__init__(self)
        self.parent_win = parent_win

        self.client = mqtt.Client()
        self.client.username_pw_set("d", "d")
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message

        self.client.connect("192.168.0.102", 1883, 60)
        self.client.subscribe(MQTT_DEVICE_SUB, qos=1)
        print("!!")

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print(client, userdata, mid, granted_qos)

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        if msg.topic == MQTT_DEVICE_SUB:
            msg_data = msg.payload.decode("utf-8")
            if msg_data == 'wait_call':
                self.parent_win.up_phone_button.setEnabled(False)
                self.parent_win.down_phone_button.setEnabled(False)

                self.parent_win.up_phone_button.setStyleSheet("color: rgb(0, 0, 0);")
                self.parent_win.down_phone_button.setStyleSheet("color: rgb(0, 0, 0);")

                self.parent_win.label.setText("Ожидание")
            elif msg_data == 'call':
                self.parent_win.label.setText("Ожидание подтверждения")
                self.client.publish(MQTT_CONTROL_PUB, "user_notify")
            elif msg_data == 'wait_user':
                self.parent_win.label.setText("ВЫЗОВ")
                self.parent_win.up_phone_button.setEnabled(True)
                self.parent_win.down_phone_button.setEnabled(True)

                self.parent_win.up_phone_button.setStyleSheet("color: rgb(255, 0, 0);")
                self.parent_win.down_phone_button.setStyleSheet("color: rgb(255, 0, 0);")
            elif msg_data == 'up_phone':
                self.parent_win.label.setText("Трубка поднята...")
            elif msg_data == 'down_phone':
                self.parent_win.label.setText('Кладу трубку...')
            elif msg_data == 'open_dor':
                self.parent_win.label.setText('Открытие двери')

    def up_phone(self):
        self.client.publish(MQTT_CONTROL_PUB, "up_phone");

    def down_phone(self):
        self.client.publish(MQTT_CONTROL_PUB, "down_phone");

    def close_dor(self):
        pass

    def __del__(self):
        self.wait()

    def run(self):
        self.client.loop_forever()

class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

        self.mqtt_th = ThreadMQTT(self)
        self.mqtt_th.start()

    def initUI(self):
        self.setGeometry(300, 300, 300, 220)
        self.setWindowTitle('Test program')

        self.label = QLabel("Ожидание")

        self.up_phone_button = QPushButton("Снять трубку")
        self.down_phone_button = QPushButton("Повесить трубку")

        self.dore_button = QPushButton("Открыть дверь")

        self.down_phone_button.setEnabled(False)
        self.up_phone_button.setEnabled(False)

        self.dore_button.setEnabled(False)

        self.up_phone_button.clicked.connect(self.up_phone_click)
        self.down_phone_button.clicked.connect(self.down_phone_click)

        self.dore_button.clicked.connect(self.open_dore_click)

        hbox = QHBoxLayout(self)
        hbox.addStretch(1)

        hbox.addWidget(self.label)
        hbox.addWidget(self.up_phone_button)
        hbox.addWidget(self.down_phone_button)

        hbox.addWidget(self.dore_button)

        self.setLayout(hbox)
        self.show()

    def up_phone_click(self):
        self.mqtt_th.up_phone()

    def down_phone_click(self):
        self.mqtt_th.down_phone()

    def open_dore_click(self):
        pass


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Window()
    sys.exit(app.exec_())
