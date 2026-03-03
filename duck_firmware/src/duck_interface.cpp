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
  last_read_time_valid_ = false;
  last_write_time_valid_ = false;

  velocity_commands_ = {0.0, 0.0};
  position_states_   = {0.0, 0.0};
  velocity_states_   = {0.0, 0.0};
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

hardware_interface::return_type DuckInterface::read(
    const rclcpp::Time &time,
    const rclcpp::Duration & /*period*/)
{
  if (!arduino_.IsDataAvailable()) {
    return hardware_interface::return_type::OK;
  }

  // Drain buffer to get the most recent reading
  std::string message;
  arduino_.ReadLine(message);
  while (arduino_.IsDataAvailable()) {
    arduino_.ReadLine(message);
  }

  // Use control loop time (consistent with joint_state_broadcaster / controllers)
  if (!last_read_time_valid_) {
    last_read_time_ = time;
    last_read_time_valid_ = true;
  }

  const double dt = (time - last_read_time_).seconds();
  last_read_time_ = time;

  if (dt <= 0.0 || dt > 1.0) {
    // guard against startup jumps or pauses
    RCLCPP_WARN(rclcpp::get_logger("DuckInterface"),
                "Skipping read integration due to abnormal dt=%.6f", dt);
    return hardware_interface::return_type::OK;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("DuckInterface"), "RX <- '%s'", message.c_str());

  std::stringstream ss(message);
  std::string res;
  int multiplier = 1;

  while (std::getline(ss, res, ',')) {
    if (res.size() < 3) { continue; }
    multiplier = (res.at(1) == 'p') ? 1 : -1;

    if (res.at(0) == 'l') {
      velocity_states_.at(0) = multiplier * std::stod(res.substr(2));  // rad/s
      position_states_.at(0) += velocity_states_.at(0) * dt;          // rad
    } else if (res.at(0) == 'r') {
      velocity_states_.at(1) = multiplier * std::stod(res.substr(2));
      position_states_.at(1) += velocity_states_.at(1) * dt;
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("DuckInterface"),
               "Parsed dt=%.3f  v_left=%.2f  v_right=%.2f  pos_l=%.2f  pos_r=%.2f",
               dt,
               velocity_states_.at(0), velocity_states_.at(1),
               position_states_.at(0), position_states_.at(1));

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DuckInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & /*period*/)
{
  if (!arduino_.IsOpen()) {
    return hardware_interface::return_type::OK;
  }

  // Throttle to ~20 Hz (50 ms)
  if (!last_write_time_valid_) {
    last_write_time_ = time;
    last_write_time_valid_ = true;
  }

  const double dt = (time - last_write_time_).seconds();
  if (dt >= 0.0 && dt < 0.05) {
    return hardware_interface::return_type::OK;  // too soon, skip
  }
  last_write_time_ = time;

  // velocity_commands_[0] = left, velocity_commands_[1] = right
  double left_cmd  = velocity_commands_.at(0);
  double right_cmd = velocity_commands_.at(1);
  std::stringstream message_stream;

  const char right_sign = (right_cmd >= 0.0) ? 'p' : 'n';
  const char left_sign  = (left_cmd  >= 0.0) ? 'p' : 'n';

  const double right_abs = std::abs(right_cmd);
  const double left_abs  = std::abs(left_cmd);

  // Keep your "leading zero when < 10" behavior
  const std::string pad_right = (right_abs < 10.0) ? "0" : "";
  const std::string pad_left  = (left_abs  < 10.0) ? "0" : "";

  message_stream << std::fixed << std::setprecision(2)
                 << "r" << right_sign << pad_right << right_abs
                 << ",l" << left_sign  << pad_left  << left_abs
                 << ",,";

  const std::string msg = message_stream.str();

  RCLCPP_DEBUG(rclcpp::get_logger("DuckInterface"),
               "TX -> '%s' (cmd right=%.2f left=%.2f)",
               msg.c_str(), right_cmd, left_cmd);

  try {
    arduino_.Write(msg);
    arduino_.DrainWriteBuffer();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("DuckInterface"),
                        "Serial write exception: " << e.what());
    return hardware_interface::return_type::ERROR;
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("DuckInterface"),
                        "Unknown error while writing to port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


}  // namespace duck_firmware

PLUGINLIB_EXPORT_CLASS(duck_firmware::DuckInterface, hardware_interface::SystemInterface)
