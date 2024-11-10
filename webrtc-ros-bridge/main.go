package main

import (
	"flag"
	"sync"

	peerconnectionchannel "github.com/3DRX/webrtc-ros-bridge/peer_connection_channel"
	signalingchannel "github.com/3DRX/webrtc-ros-bridge/signaling_channel"
	"github.com/pion/webrtc/v4"
)

func main() {
	addr := flag.String("addr", "localhost:8080", "http service address")
	var candidateMux sync.Mutex
	sdpChan := make(chan webrtc.SessionDescription)
	candidateChan := make(chan webrtc.ICECandidateInit, 5)
	candidateToPeerChan := make(chan *webrtc.ICECandidate, 5)
	flag.Parse()
	sc := signalingchannel.InitSignalingChannel(
		addr,
		sdpChan,
		candidateChan,
		candidateToPeerChan,
		&candidateMux,
	)
	go sc.Spin()
	pc := peerconnectionchannel.InitPeerConnectionChannel(
		sdpChan,
		candidateChan,
		candidateToPeerChan,
		&candidateMux,
	)
	go pc.Spin()
	<-sc.Done()
}
