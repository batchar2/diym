package main

import (
	"github.com/stianeikeland/go-rpio"
	//"fmt"
	"time"
	opt "./options"
)

const (
	numberIteration = 5
	thresholdValueCall  = 1
	thresholdValueWait  = 0
)

func GpioNotification(pin rpio.Pin, c chan <- int) {
	var state = opt.EVENT_UNDEFINED

	for {
		var jitterState [numberIteration]int
		for i := 0; i < numberIteration; i++ {
			var currentValue = -1
			if v := pin.Read(); rpio.High == v {
				currentValue = 0
			} else {
				currentValue = 1
			}
			jitterState[i] = currentValue
			time.Sleep(time.Second/10/numberIteration)
		}

		var summaryValue = 0
		for i := 0; i < numberIteration; i++ {
			summaryValue += jitterState[i]
			//fmt.Print(" ", jitterState[i])
		}
		//fmt.Println("")
		if (summaryValue >= thresholdValueCall && state != opt.EVENT_CALL) {
			c <- opt.EVENT_CALL
			state = opt.EVENT_CALL
		} else if (summaryValue <= thresholdValueWait && state != opt.EVENT_WAIT_CALL) {
			c <- opt.EVENT_WAIT_CALL
			state = opt.EVENT_WAIT_CALL
		}
	}
}
