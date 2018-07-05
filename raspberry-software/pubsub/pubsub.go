package pubsub
import (
	mqtt "github.com/eclipse/paho.mqtt.golang"
	"fmt"
	opt "../options"
)

type PubSub struct {
	Server string

	client mqtt.Client
}

func (self* PubSub)Init() bool {

	opts := mqtt.NewClientOptions().AddBroker(self.Server)

	self.client = mqtt.NewClient(opts)

	if token := self.client.Connect(); token.Wait() && token.Error() != nil {
		fmt.Println("Error connection")
		return false
	}
	return true;
}

func (self* PubSub) sendMessage(msg string) bool {
	if token := self.client.Publish(opt.MQTT_DEVICE_PUB, 0, false, msg); token.Wait() && token.Error() != nil {
		fmt.Println(token.Error())
		return false
	}
	return true;
}

func (self* PubSub) WaitLoop(stateDevice <-chan int, manualNotif chan <-int) {

	/*
	EVENT_UNDEFINED 	= -1 	// неизвестно
	EVENT_WAIT_CALL 	= 2		// ожидаем вызов
	EVENT_CALL 			= 1		// пошел вызов
	EVENT_NOTIF 		= 3		// польззователь получил вызов
	EVENT_USER_ACTION	= 4		// ждем действие от пользователя
	EVENT_UP_PHONE 		= 5
	EVENT_DOWN_PHONE 	= 6
	EVENT_OPEN_DOORE 	= 7

	MQTT_DEVICE_WAIT_CALL_NOTIF_STATE 	= "wait_call"
	MQTT_DEVICE_ACTION_CALL_STATE 		= "action_call"
	MQTT_USER_WAIT_NOTIF_STATE 			= "wait_user_notif"
	MQTT_DEVICE_ACTION_USER_ACTION		= "action_user_mancontrol"
	MQTT_USER_ACTION_UP_HONE_STATE 		= "action_up_phone"
	MQTT_USER_ACTION_DOWN_PHONE_STATE 	= "action_down_phone"
	MQTT_USER_ACTION_OPEN_DOOR_STATE 	= "action_open_door"

	 */

	if token := self.client.Subscribe(opt.MQTT_CONTROL_SUB, 0, func(client mqtt.Client, message mqtt.Message) {
		payload := string(message.Payload())
		if payload == opt.MQTT_USER_WAIT_NOTIF_STATE {
			manualNotif <- opt.EVENT_NOTIF
		} else if payload == opt.MQTT_USER_ACTION_UP_HONE_STATE {
			manualNotif <- opt.EVENT_UP_PHONE
		} else if payload == opt.MQTT_USER_ACTION_DOWN_PHONE_STATE {
			manualNotif <- opt.EVENT_DOWN_PHONE
		} else if payload == opt.MQTT_USER_ACTION_OPEN_DOOR_STATE {
			manualNotif <- opt.EVENT_OPEN_DOORE
		} else {
			fmt.Printf("Error: undefined command - %s\n", payload)
		}
	}); token.Wait() && token.Error() != nil {
		fmt.Println(token.Error())
	}

	for {
		select {
		case msg := <- stateDevice:
			if (msg == opt.EVENT_WAIT_CALL) {
				self.sendMessage(opt.MQTT_DEVICE_WAIT_CALL_NOTIF_STATE)
			} else if (msg == opt.EVENT_CALL) {
				self.sendMessage(opt.MQTT_DEVICE_ACTION_CALL_STATE)
			}
		}
	}
}
