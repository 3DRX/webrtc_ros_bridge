#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "webrtc/api/audio_codecs/builtin_audio_decoder_factory.h"
#include "webrtc/api/audio_codecs/builtin_audio_encoder_factory.h"
#include "webrtc/api/create_peerconnection_factory.h"
#include "webrtc/api/peer_connection_interface.h"
#include "webrtc/media/engine/internal_decoder_factory.h"
#include "webrtc/media/engine/internal_encoder_factory.h"
#include "webrtc/media/engine/multiplex_codec_factory.h"
#include "webrtc/rtc_base/thread.h"

namespace webrtc_ros {

class WebRTCNode : public rclcpp::Node {
 public:
  explicit WebRTCNode(const rclcpp::NodeOptions& options)
      : Node("webrtc_ros_client_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing WebRTC Node");

    // Create WebRTC peer connection factory
    rtc::Thread* network = rtc::Thread::CreateWithSocketServer().release();
    rtc::Thread* worker = rtc::Thread::Create().release();
    rtc::Thread* signaling = rtc::Thread::Create().release();
    network->Start();
    worker->Start();
    signaling->Start();

    peer_connection_factory_ = webrtc::CreatePeerConnectionFactory(
        std::move(network), std::move(worker), std::move(signaling),
        nullptr,                                     // AudioDeviceModule
        webrtc::CreateBuiltinAudioEncoderFactory(),  // AudioEncoderFactory
        webrtc::CreateBuiltinAudioDecoderFactory(),  // AudioDecoderFactory
        std::unique_ptr<webrtc::VideoEncoderFactory>(
            new webrtc::MultiplexEncoderFactory(
                std::make_unique<webrtc::InternalEncoderFactory>())),
        std::unique_ptr<webrtc::VideoDecoderFactory>(
            new webrtc::MultiplexDecoderFactory(
                std::make_unique<webrtc::InternalDecoderFactory>())),
        nullptr,  // AudioMixer
        nullptr   // AudioProcessing
    );

    if (!peer_connection_factory_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to create PeerConnectionFactory");
      return;
    }

    // Initialize PeerConnection (leave signaling blank for now)
    CreatePeerConnection();
  }

 private:
  void CreatePeerConnection() {
    // Configure the PeerConnection (placeholder for signaling logic)
    webrtc::PeerConnectionInterface::RTCConfiguration config;
    webrtc::PeerConnectionDependencies dependencies(
        nullptr);  // Use default or your own observer

    rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection =
        peer_connection_factory_->CreatePeerConnection(config,
                                                       std::move(dependencies));

    if (!peer_connection) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create PeerConnection");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "PeerConnection created successfully");
    // Add further configurations or media stream setups here
  }

  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface>
      peer_connection_factory_;
};

}  // namespace webrtc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(webrtc_ros::WebRTCNode)
