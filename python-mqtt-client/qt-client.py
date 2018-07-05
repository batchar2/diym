#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal
import paho.mqtt.client as mqtt


MQTT_CONTROL_PUB = '/dm/state/control'
MQTT_DEVICE_SUB  = '/dm/state/device'

class ThreadMQTT(QThread):
    wait_call = pyqtSignal()
    call = pyqtSignal()
    wait_user = pyqtSignal()
    up_phone = pyqtSignal()
    down_phone= pyqtSignal()
    open_dor = pyqtSignal()

    def __init__(self, parent_win):
        QThread.__init__(self)
        self.parent_win = parent_win

        self.client = mqtt.Client()
        self.client.username_pw_set("d", "d")
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message

        self.client.connect("192.168.0.101", 1883, 60)
        self.client.subscribe(MQTT_DEVICE_SUB, qos=1)
        print("!!")

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print(client, userdata, mid, granted_qos)

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        if msg.topic == MQTT_DEVICE_SUB:
            msg_data = msg.payload.decode("utf-8")
            if msg_data == 'wait_call':

                self.wait_call.emit()
            elif msg_data == 'wait_call':
                self.call.emit()
                self.client.publish(MQTT_CONTROL_PUB, "user_notify")
            elif msg_data == 'action_user_mancontrol':
                self.wait_user.emit()
            elif msg_data == 'up_phone':
                self.up_phone.emit()
            elif msg_data == 'down_phone':
                self.down_phone.emit()
            elif msg_data == 'open_dor':
                self.open_dor.emit()

    def up_phone_send(self):
        self.client.publish(MQTT_CONTROL_PUB, "up_phone");

    def down_phone_send(self):
        self.client.publish(MQTT_CONTROL_PUB, "down_phone");

    def open_dore_send(self):
        self.client.publish(MQTT_CONTROL_PUB, "open_dor");

    def __del__(self):
        self.wait()

    def run(self):
        self.client.loop_forever()

class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.mqtt_th = ThreadMQTT(self)
        self.mqtt_th.start()

        self.initUI()

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

        self.mqtt_th.wait_call.connect(self.wait_call)
        self.mqtt_th.call.connect(self.call)
        self.mqtt_th.wait_user.connect(self.wait_user)
        self.mqtt_th.up_phone.connect(self.up_phone)
        self.mqtt_th.down_phone.connect(self.down_phone)
        self.mqtt_th.open_dor.connect(self.open_dor)

        hbox = QHBoxLayout(self)
        hbox.addStretch(1)

        hbox.addWidget(self.label)
        hbox.addWidget(self.up_phone_button)
        hbox.addWidget(self.down_phone_button)

        hbox.addWidget(self.dore_button)

        self.setLayout(hbox)
        self.show()

    def wait_call(self):
        self.up_phone_button.setEnabled(False)
        self.down_phone_button.setEnabled(False)
        self.dore_button.setEnabled(False)

        self.up_phone_button.setStyleSheet("color: rgb(0, 0, 0);")
        self.down_phone_button.setStyleSheet("color: rgb(0, 0, 0);")
        self.dore_button.setStyleSheet("color: rgb(0, 0, 0);")

        self.label.setText("Ожидание")

    def call(self):
        self.label.setText("Ожидание подтверждения")

    def wait_user(self):
        self.label.setText("ВЫЗОВ")
        self.up_phone_button.setEnabled(True)
        self.up_phone_button.setStyleSheet("color: rgb(255, 0, 0);")


    def up_phone(self):
        self.label.setText("Трубка поднята...")
        self.up_phone_button.setEnabled(False)
        self.up_phone_button.setStyleSheet("color: rgb(0, 0, 0);")

        self.down_phone_button.setEnabled(True)
        self.down_phone_button.setStyleSheet("color: rgb(0, 255, 0);")

        self.dore_button.setEnabled(True)
        self.dore_button.setStyleSheet("color: rgb(0, 0, 255);")

    def down_phone(self):
        self.label.setText('Кладу трубку...')
        self.up_phone_button.setEnabled(False)
        self.down_phone_button.setEnabled(False)
        self.dore_button.setEnabled(False)

        self.up_phone_button.setStyleSheet("color: rgb(0, 0, 0);")
        self.down_phone_button.setStyleSheet("color: rgb(0, 0, 0);")
        self.dore_button.setStyleSheet("color: rgb(0, 0, 0);")

    def open_dor(self):
        self.label.setText('Открытие двери')
        self.mqtt_th.down_phone_send()

    def up_phone_click(self):
        print(">>> up_phone_click")
        self.mqtt_th.up_phone_send()

    def down_phone_click(self):
        print(">>> down_phone_click")
        self.mqtt_th.down_phone_send()

    def open_dore_click(self):
        print(">>> open_dore_click")
        self.mqtt_th.open_dore_send()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Window()
    sys.exit(app.exec_())
