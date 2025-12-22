#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include <chrono>
#include <string>

using namespace std::chrono_literals;
using LibSerial::SerialPort;
using LibSerial::BaudRate;

class SerialSenderNode : public rclcpp::Node {
public:
  SerialSenderNode() : Node("serial_sender")
  {
    // Declare parameters (with defaults)
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = this->declare_parameter<int>("baud", 115200);
    payload_ = this->declare_parameter<std::string>("payload", "on\n");

    RCLCPP_INFO(get_logger(), "Opening serial port %s @ %d", port_.c_str(), baud_);

    try {
      serial_.Open(port_);
      // Map int -> LibSerial baud
      switch (baud_) {
        case 9600:   serial_.SetBaudRate(BaudRate::BAUD_9600); break;
        case 19200:  serial_.SetBaudRate(BaudRate::BAUD_19200); break;
        case 38400:  serial_.SetBaudRate(BaudRate::BAUD_38400); break;
        case 57600:  serial_.SetBaudRate(BaudRate::BAUD_57600); break;
        case 115200: serial_.SetBaudRate(BaudRate::BAUD_115200); break;
        default:
          RCLCPP_WARN(get_logger(), "Unsupported baud %d, falling back to 115200", baud_);
          serial_.SetBaudRate(BaudRate::BAUD_115200);
      }
      serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Failed to open/configure %s: %s", port_.c_str(), e.what());
      throw;
    }

    // Send once shortly after startup (gives USB CDC a moment to settle)
    timer_ = this->create_wall_timer(200ms, [this]() {
      try {
        serial_.Write(payload_);
        serial_.DrainWriteBuffer();
        RCLCPP_INFO(this->get_logger(), "Wrote to %s: '%s'", port_.c_str(), payload_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Write failed: %s", e.what());
      }
      // cancel after one-shot
      timer_->cancel();
    });
  }

  ~SerialSenderNode() override {
    try {
      if (serial_.IsOpen()) serial_.Close();
    } catch (...) {
      // ignore
    }
  }

private:
  SerialPort serial_;
  std::string port_;
  int baud_;
  std::string payload_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SerialSenderNode>());
  } catch (...) {
    // already logged
  }
  rclcpp::shutdown();
  return 0;
}
