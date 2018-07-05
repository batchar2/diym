package pins

import (
	"fmt"
	"github.com/stianeikeland/go-rpio"
)

const (
	GPIO_CALL = 4
	GPIO_UP_PONE = 5
	GPIO_OPEN_DOOR = 6
)

type GpioPins struct {
	InSignalCall rpio.Pin
	OutUpPhone   rpio.Pin
	OutOpenDoor  rpio.Pin
}

func (self *GpioPins) Init() bool {
	if err := rpio.Open(); err != nil {
		fmt.Println(err)
		return false
	}
	self.InSignalCall = rpio.Pin(GPIO_CALL)
	self.InSignalCall.Input()
	self.InSignalCall.PullUp()

	self.OutUpPhone = rpio.Pin(GPIO_UP_PONE)
	self.OutUpPhone.Output()

	self.OutOpenDoor = rpio.Pin(GPIO_OPEN_DOOR)
	self.OutOpenDoor.Output()

	return true
}

func (self *GpioPins) UInit() {
	rpio.Close()
}
