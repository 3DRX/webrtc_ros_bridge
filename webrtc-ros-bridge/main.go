package main

import (
	"flag"

	signalingchannel "github.com/3DRX/webrtc-ros-bridge/signaling_channel"
)

var addr = flag.String("addr", "localhost:8080", "http service address")

func main() {
	sc := signalingchannel.InitSignalingChannel(*addr)
	go sc.Spin()
	<-sc.Done()
}
