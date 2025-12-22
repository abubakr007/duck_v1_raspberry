#include "duck_firmware/duck_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iomanip>  // for std::setprecision

namespace duck_firmware
{

DuckInterface::DuckInterface() {}
DuckInterface::~DuckInterface() {
  if (arduino_.IsOpen()) {
    try {
      arduino_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("DuckInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn DuckInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(rclcpp::get_logger("DuckInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"),
              "Initializing hardware. Port: %s, joints: %zu",
              port_.c_str(), info_.joints.size());

  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DuckInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"),
              "Exported %zu state interfaces (position, velocity per joint).",
              state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DuckInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"),
              "Exported %zu command interfaces (velocity).",
              command_interfaces.size());
  return command_interfaces;
}

CallbackReturn DuckInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"), "Starting robot hardware ...");

  velocity_commands_ = {0.0, 0.0};
  position_states_   = {0.0, 0.0};
  velocity_states_   = {0.0, 0.0};
  last_write_ = rclcpp::Clock().now();
  try {
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      
      // Add these lines for proper serial configuration:
      arduino_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      arduino_.SetParity(LibSerial::Parity::PARITY_NONE);
      arduino_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      arduino_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
      
      // Wait for Arduino to reset (if it resets on serial connection)
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      
      // Flush any startup data
      arduino_.FlushInputBuffer();
      arduino_.FlushOutputBuffer();
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("DuckInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"),
              "Hardware started, ready to take commands on %s @115200", port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DuckInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"), "Stopping robot hardware ...");
  if (arduino_.IsOpen()) {
    try {
      arduino_.Close();
    } catch (...) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("DuckInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DuckInterface::read(const rclcpp::Time &,
                                                    const rclcpp::Duration &)
{
  // Print when a line is received from the Arduino
  if (arduino_.IsDataAvailable()) {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);

    // --- RX LOG: raw line from Arduino ---
    RCLCPP_INFO(rclcpp::get_logger("DuckInterface"), "RX <- '%s'", message.c_str());

    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;

    while (std::getline(ss, res, ',')) {
      if (res.size() < 3) { continue; }  // minimal guard
      multiplier = res.at(1) == 'p' ? 1 : -1;

      if (res.at(0) == 'l') {
        velocity_states_.at(0) = multiplier * std::stod(res.substr(2));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      } else if (res.at(0) == 'r') {
        velocity_states_.at(1) = multiplier * std::stod(res.substr(2));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }

    // --- RX LOG: parsed states summary ---
    RCLCPP_INFO(rclcpp::get_logger("DuckInterface"),
                "Parsed dt=%.3f  v_right=%.2f  v_left=%.2f  pos_r=%.2f  pos_l=%.2f",
                dt,
                velocity_states_.at(0), velocity_states_.at(1),
                position_states_.at(0), position_states_.at(1));

    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type DuckInterface::write(const rclcpp::Time &,
                                                     const rclcpp::Duration &)
{
  // Throttle writes to ~50ms (twice per Arduino cycle to stay responsive)
  auto now = rclcpp::Clock().now();
  auto time_since_last_write = (now - last_write_).seconds();
  
  if (time_since_last_write < 0.05) {  // 50ms = 0.05s
    return hardware_interface::return_type::OK;  // Skip this write
  }
  
  last_write_ = now;
  // In the write function, before building the message
// Temporarily swap to test
double temp_right_cmd = velocity_commands_.at(0);
velocity_commands_.at(0) = velocity_commands_.at(1); // Assign left cmd to right
velocity_commands_.at(1) = temp_right_cmd;           // Assign right cmd to left

// ... then proceed with the rest of the function as normal

  // Build the message
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
  std::string compensate_zeros_right = (std::abs(velocity_commands_.at(0)) < 10.0) ? "0" : "";
  std::string compensate_zeros_left  = (std::abs(velocity_commands_.at(1)) < 10.0) ? "0" : "";

  message_stream << std::fixed << std::setprecision(2)
                 << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0))
                 << ",l" << left_wheel_sign << compensate_zeros_left  << std::abs(velocity_commands_.at(1))
                 << ",,";

  const auto msg = message_stream.str();

  RCLCPP_INFO(rclcpp::get_logger("DuckInterface"),
              "TX -> '%s'   (cmd v_right=%.2f, v_left=%.2f)",
              msg.c_str(), velocity_commands_.at(0), velocity_commands_.at(1));

  try {
    arduino_.Write(msg);
    arduino_.DrainWriteBuffer();
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("DuckInterface"),
                        "Something went wrong while sending the message "
                        << msg << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace duck_firmware

PLUGINLIB_EXPORT_CLASS(duck_firmware::DuckInterface, hardware_interface::SystemInterface)
