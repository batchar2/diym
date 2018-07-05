package main

import (
	//"fmt"
	pins "./pins"
	opt "./options"
	pubsub "./pubsub"
	//"github.com/stianeikeland/go-rpio"
	//"time"
	"fmt"
)
/*
func EventLoop(gpioNotif <- chan int, manualNotif <- chan int, deviceControl chan <- int) {//, pinUpPhone rpio.Pin, pinOpenDoor rpio.Pin) {
	state := opt.EVENT_UNDEFINED
	for {
		select {
		case gpio := <- gpioNotif:
			if gpio == opt.EVENT_WAIT_CALL {
				//UpPhoneChannel <- EVENT_DOWN_PONE
				//pinUpPhone.Low()
				//pinOpenDoor.Low()
				//fmt.Println("WAIT CALL")
				deviceControl <- gpio; state = gpio
			} else if gpio == opt.EVENT_CALL {
				//UpPhoneChannel <- EVENT_UP_PHONE
				//time.Sleep(time.Second*3)
				//pinUpPhone.High()
				//time.Sleep(time.Second/2)
				//pinOpenDoor.High()
				//fmt.Println("CALL")
				deviceControl <- gpio; state = gpio
			}
		case manual := <- manualNotif:
			if state != opt.EVENT_WAIT_CALL {
				if manual == opt.EVENT_NOTIF {
					deviceControl <- manual; state = manual
				}
			}
			if state == opt.EVENT_NOTIF {
				if manual == opt.EVENT_UP_PHONE {
					deviceControl <- manual; state = manual
				} else if manual == opt.EVENT_DOWN_PHONE {
					deviceControl <- manual; state = manual
				} else if manual == opt.EVENT_DOWN_PHONE {
					deviceControl <- manual; state = manual
				}
			}
		}
	}
}
*/
/*
func UpPhone(pin rpio.Pin, UpPhoneChannel <- chan int) {
	select {
	case notif := <- UpPhoneChannel:
		if (notif == EVENT_UP_PHONE) {
			fmt.Println("EVENT_UP_PHONE")
			//pin.High()
		} else if (notif == EVENT_DOWN_PONE) {
			fmt.Println("EVENT_DOWN_PONE")
			//pin.Low()
		}
	}
}
*/

func GpioControl(deviceControlChannel chan int, pins pins.GpioPins){
	for {
		select {
		case state := <- deviceControlChannel:
			if state == opt.EVENT_WAIT_CALL || state == opt.EVENT_CALL || state == opt.EVENT_NOTIF || state == opt.EVENT_UNDEFINED {
				fmt.Println("WAIT/CALL")
				pins.OutUpPhone.Low()
				pins.OutOpenDoor.Low()
				//pinOpenDoor.Low()
			} else if state == opt.EVENT_UP_PHONE {
				fmt.Println("UP")
				pins.OutUpPhone.High()
			} else if state == opt.EVENT_OPEN_DOORE {
				pins.OutOpenDoor.High()
			} else if state == opt.EVENT_DOWN_PHONE {
				pins.OutOpenDoor.Low()
				pins.OutOpenDoor.Low()
			}
		}
	}
}


func main() {
	var gpio = pins.GpioPins{}
	gpio.Init()
	defer gpio.UInit()

	var pubsub = pubsub.PubSub{Server:"192.168.0.101:1883"}
	pubsub.Init()

	// Стостояние пинов
	var gpioNotifChannel chan int = make(chan int)
	// Ручное управление по mqtt
	var manualNotifChannel chan int = make(chan int)
	// Управление реле
	var deviceControlChannel chan int = make(chan int)
	// Публикация состояние устройства
	var publicStateDeviceChannel chan int = make(chan int)

	// Получение состояние устройства
	go GpioNotification(gpio.InSignalCall, gpioNotifChannel)
	// Получение команд от пользователя
	go pubsub.WaitLoop(publicStateDeviceChannel, manualNotifChannel)
	// управление реле
	go GpioControl(deviceControlChannel, gpio)

	state := opt.EVENT_UNDEFINED
	for {
		select {
		case gpio := <- gpioNotifChannel:
			if gpio == opt.EVENT_WAIT_CALL {
				deviceControlChannel <- gpio; state = gpio
			} else if gpio == opt.EVENT_CALL {
				deviceControlChannel <- gpio; state = gpio
			}
		case manual := <- manualNotifChannel:
			if state != opt.EVENT_WAIT_CALL {
				if manual == opt.EVENT_NOTIF {
					deviceControlChannel <- manual; state = manual
				}
			}
			if state == opt.EVENT_NOTIF {
				if manual == opt.EVENT_UP_PHONE {
					deviceControlChannel <- manual; state = manual
				} else if manual == opt.EVENT_DOWN_PHONE {
					deviceControlChannel <- manual; state = manual
				} else if manual == opt.EVENT_DOWN_PHONE {
					deviceControlChannel <- manual; state = manual
				}
			}
		}
	}
}