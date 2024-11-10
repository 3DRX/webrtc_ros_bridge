package signalingchannel

import (
	"encoding/json"
	"log/slog"
	"net/url"
	"strconv"

	"github.com/gorilla/websocket"
	"golang.org/x/exp/rand"
)

type SignalingChannel struct {
	addr string
	done chan struct{}
	recv chan []byte
	c    *websocket.Conn
}

type signalingResponse struct {
	Sdp  string
	Type string
}

func InitSignalingChannel(addr string) *SignalingChannel {
	return &SignalingChannel{
		addr: addr,
		done: make(chan struct{}),
		recv: make(chan []byte),
		c:    nil,
	}
}

func newStreamId() string {
	return "webrtc_ros-stream-" + strconv.Itoa(rand.Intn(1000000000))
}

func composeActions() map[string]interface{} {
	streamId := newStreamId()
	action := map[string]interface{}{
		"type": "configure",
		"actions": []map[string]interface{}{
			{
				"type": "add_stream",
				"id":   streamId,
			},
			{
				"type":      "add_video_track",
				"stream_id": streamId,
				"id":        streamId + "/subscribed_video",
				"src":       "ros_image:/image",
			},
		},
	}
	return action
}

func toTextMessage(data map[string]interface{}) ([]byte, error) {
	jsonData, err := json.Marshal(data)
	if err != nil {
		return nil, err
	}
	return jsonData, nil
}

func (s *SignalingChannel) Spin() {
	defer close(s.done)
	u := url.URL{Scheme: "ws", Host: s.addr, Path: "/webrtc"}
	slog.Info("start spinning", "url", u.String())
	c, _, err := websocket.DefaultDialer.Dial(u.String(), nil)
	defer c.Close()
	if err != nil {
		slog.Error("dial error", "error", err)
		return
	}
	go func() {
		for {
			_, message, err := c.ReadMessage()
			if err != nil {
				slog.Error("recv error", "err", err)
				return
			}
			slog.Info("recv success", "msg", message)
			s.recv <- message
		}
	}()
	slog.Info("dial success")

	cfgMessage, err := toTextMessage(composeActions())
	if err != nil {
		slog.Error("compose message error", "error", err)
		return
	}
	c.WriteMessage(websocket.TextMessage, cfgMessage)
	recvRaw := <-s.recv
	resp := signalingResponse{}
	err = json.Unmarshal(recvRaw, &resp)
	if err != nil {
		slog.Error("unmarshal error", "error", err)
		return
	}
	if resp.Type != "offer" {
		slog.Error("unexpected response", "type", resp.Type)
		return
	}
	slog.Info("offer", "sdp", resp.Sdp)
	for {
	}
}

func (s *SignalingChannel) Done() chan struct{} {
	return s.done
}
