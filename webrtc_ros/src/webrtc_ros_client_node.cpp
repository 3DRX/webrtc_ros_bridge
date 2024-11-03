#include <cv_bridge/cv_bridge.h>
#include <webrtc_ros/ros_video_capturer.h>
#include <webrtc_ros/webrtc_client.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <webrtc_ros_msgs/srv/get_ice_servers.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

using namespace std::chrono_literals;

namespace webrtc_ros {

typedef websocketpp::client<websocketpp::config::asio_client> client;

cv::Mat ConvertVideoFrameToMat(const webrtc::VideoFrame& frame) {
  // Implement the conversion from WebRTC VideoFrame to OpenCV Mat
  // This is a placeholder implementation
  int width = frame.width();
  int height = frame.height();
  cv::Mat image(height, width, CV_8UC3);  // Assuming a 3-channel image
  // Fill the image with data from the frame
  // ...
  return image;
}

class WebrtcRosClientNode : public rclcpp::Node {
 public:
  WebrtcRosClientNode()
      : Node("webrtc_ros_client_node"), signaling_channel_(nullptr) {
    // Initialize WebRTC client

    // Establish WebSocket connection to the server node
    establishWebSocketConnection();

    // Create peer connection to receive video stream
    createPeerConnection();

    // Publisher for the image topic
    image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
  }

 private:
  void establishWebSocketConnection() {
    try {
      // Set logging to be pretty verbose (everything except message payloads)
      c_.set_access_channels(websocketpp::log::alevel::all);
      c_.clear_access_channels(websocketpp::log::alevel::frame_payload);

      // Initialize ASIO
      c_.init_asio();

      // Register our message handler
      c_.set_message_handler(bind(&WebrtcRosClientNode::onMessage, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));

      // Create a connection to the given URI and queue it for connection once
      // the event loop starts
      websocketpp::lib::error_code ec;
      client::connection_ptr con = c_.get_connection("ws://localhost:8080/webrtc", ec);
      if (ec) {
        RCLCPP_ERROR(this->get_logger(),
                     "Could not create connection because: %s",
                     ec.message().c_str());
        return;
      }

      // Note that connect here only requests a connection. No network messages
      // are exchanged until the event loop starts running in the next line.
      c_.connect(con);

      // Start the ASIO io_service run loop
      c_.run();
    } catch (websocketpp::exception const& e) {
      RCLCPP_ERROR(this->get_logger(), "WebSocket exception: %s", e.what());
    }
  }

  void onMessage(websocketpp::connection_hdl hdl, client::message_ptr msg) {
    RCLCPP_INFO(this->get_logger(), "Received message: %s",
                msg->get_payload().c_str());
    // On Siganling Message
  }

  void createPeerConnection() {
    // Implement peer connection creation to receive video stream
    // This is a placeholder for actual peer connection code
    RCLCPP_INFO(this->get_logger(),
                "Peer connection created to receive video stream");
  }

  void onVideoFrameReceived(const webrtc::VideoFrame& frame) {
    // Convert WebRTC video frame to ROS image message
    cv::Mat image = ConvertVideoFrameToMat(frame);
    auto msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  SignalingChannel* signaling_channel_;
  client c_;
};

}  // namespace webrtc_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<webrtc_ros::WebrtcRosClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
