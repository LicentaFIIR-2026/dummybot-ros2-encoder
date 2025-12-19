#include "dummybot_hardware_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

namespace dummybot_hardware
{

// ============================================================================
// INITIALIZATION
// ============================================================================

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    // Call base class on_init
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Initializing...");

    // Read parameters from URDF
    serial_port_ = info_.hardware_parameters["serial_port"];
    serial_baud_ = std::stoi(info_.hardware_parameters["serial_baud"]);
    wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
    wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
    ticks_per_revolution_ = std::stoi(info_.hardware_parameters["ticks_per_revolution"]);

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Parameters: port=%s, baud=%d, wheel_radius=%.4f, wheel_sep=%.4f, ticks_per_rev=%d",
                serial_port_.c_str(), serial_baud_, wheel_radius_, wheel_separation_, ticks_per_revolution_);

    // Initialize joint configurations
    joint_configs_.clear();
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        JointConfig config;
        config.name = info_.joints[i].name;
        
        // Map joint names to encoder indices
        if (config.name.find("front_left") != std::string::npos) {
            config.encoder_index = 0;  // FL
        } else if (config.name.find("front_right") != std::string::npos) {
            config.encoder_index = 1;  // FR
        } else if (config.name.find("rear_left") != std::string::npos) {
            config.encoder_index = 2;  // RL
        } else if (config.name.find("rear_right") != std::string::npos) {
            config.encoder_index = 3;  // RR
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                        "Unknown joint name: %s", config.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        config.calibration_factor = 1.0;  // Default, will load from ESP32
        joint_configs_.push_back(config);
    }

    // Initialize state vectors
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_efforts_.resize(info_.joints.size(), 0.0);
    last_encoder_counts_.resize(4, 0);
    calibration_factors_.resize(4, 1.0);
    
    first_read_ = true;
    serial_fd_ = -1;

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Initialized with %zu joints", info_.joints.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Configuring...");

    // Open serial port
    serial_fd_ = init_serial(serial_port_, serial_baud_);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Failed to open serial port: %s", serial_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Serial port opened: %s", serial_port_.c_str());

    // Wait for ESP32 to be ready
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Load calibration from ESP32 (optional, falls back to 1.0)
    if (!load_calibration_from_esp32()) {
        RCLCPP_WARN(rclcpp::get_logger("DummyBotHardwareInterface"),
                   "Could not load calibration from ESP32, using defaults");
    }

    // Reset encoders
    if (!reset_encoders()) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Failed to reset encoders");
       // return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Configuration complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// STATE AND COMMAND INTERFACES
// ============================================================================

std::vector<hardware_interface::StateInterface> 
DummyBotHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
               "Exported %zu state interfaces", state_interfaces.size());

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DummyBotHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
               "Exported %zu command interfaces", command_interfaces.size());

    return command_interfaces;
}

// ============================================================================
// ACTIVATION / DEACTIVATION
// ============================================================================

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Activating...");
    
    // Reset state
    first_read_ = true;
    last_read_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Deactivating...");
    
    // Stop motors
    send_motor_speeds(0, 0, 0, 0);
    
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// READ / WRITE
// ============================================================================

hardware_interface::return_type DummyBotHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    // Read encoder values from ESP32
    std::vector<int32_t> encoder_counts = read_encoders();
    
    if (encoder_counts.size() != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Failed to read encoders");
        return hardware_interface::return_type::ERROR;
    }

    // Calculate time delta
    rclcpp::Duration dt = time - last_read_time_;
    double dt_seconds = dt.seconds();

    if (first_read_) {
        // First read, just store positions
        last_encoder_counts_ = encoder_counts;
        first_read_ = false;
    } else {
        // Calculate position and velocity for each joint
        for (size_t i = 0; i < joint_configs_.size(); ++i)
        {
            int encoder_idx = joint_configs_[i].encoder_index;
            
            // Position: convert ticks to radians
            hw_positions_[i] = encoder_ticks_to_radians(encoder_counts[encoder_idx]);
            
            // Velocity: delta_position / delta_time
            if (dt_seconds > 0.0) {
                int32_t delta_ticks = encoder_counts[encoder_idx] - last_encoder_counts_[encoder_idx];
                double delta_rad = encoder_ticks_to_radians(delta_ticks);
                hw_velocities_[i] = delta_rad / dt_seconds;
            }
        }
        
        last_encoder_counts_ = encoder_counts;
    }

    last_read_time_ = time;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyBotHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Convert commanded velocities (rad/s) to motor speeds (ticks/frame)
    std::vector<int32_t> motor_speeds(4, 0);
    
    for (size_t i = 0; i < joint_configs_.size(); ++i)
    {
        int encoder_idx = joint_configs_[i].encoder_index;
        motor_speeds[encoder_idx] = radians_per_sec_to_ticks_per_frame(hw_commands_[i]);
    }

    // ADAUGĂ ACEST LOG:
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Motor speeds: FL=%d FR=%d RL=%d RR=%d", 
                motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);

    // Send to ESP32: m FL FR RL RR
    if (!send_motor_speeds(motor_speeds[0], motor_speeds[1], 
                          motor_speeds[2], motor_speeds[3])) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Failed to send motor speeds");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

// ============================================================================
// SERIAL COMMUNICATION HELPERS
// ============================================================================

int DummyBotHardwareInterface::init_serial(const std::string& port_name, int baud_rate)
{
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate
    speed_t speed;
    switch(baud_rate) {
        case 9600: speed = B9600; break;
        case 115200: speed = B115200; break;
        default: speed = B115200; break;
    }
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // No flow control
    options.c_cflag &= ~CRTSCTS;
    
    // Enable receiver, ignore modem control lines
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output
    options.c_oflag &= ~OPOST;
    
    // No input processing
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // Timeouts
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;  // 0.1 second timeout
    
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);
    
    return fd;
}

void DummyBotHardwareInterface::close_serial()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool DummyBotHardwareInterface::send_command(const std::string& cmd)
{
    if (serial_fd_ < 0) return false;
    
    std::string full_cmd = cmd + "\n";
    ssize_t bytes_written = ::write(serial_fd_, full_cmd.c_str(), full_cmd.length());
    
    if (bytes_written != static_cast<ssize_t>(full_cmd.length())) {
        return false;
    }
    
    tcdrain(serial_fd_);  // Wait for transmission to complete
    return true;
}

std::string DummyBotHardwareInterface::read_response(int timeout_ms)
{
    if (serial_fd_ < 0) return "";
    
    std::string response;
    char buffer[256];
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
        
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            response += buffer;
            
            // Check if we have a complete line
            if (response.find('\n') != std::string::npos) {
                // Remove trailing newline
                if (response.back() == '\n') response.pop_back();
                if (response.back() == '\r') response.pop_back();
                break;
            }
        }
        
        // Check timeout
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        
        if (elapsed.count() > timeout_ms) {
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    return response;
}

// ============================================================================
// ESP32 PROTOCOL HELPERS
// ============================================================================

bool DummyBotHardwareInterface::reset_encoders()
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Resetting encoders...");
    
    if (!send_command("r")) {
        return false;
    }
    
    std::string response = read_response(200);
    
    if (response.find("OK") != std::string::npos) {
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), "Encoders reset");
        return true;
    }
    
    return false;
}

std::vector<int32_t> DummyBotHardwareInterface::read_encoders()
{
    std::vector<int32_t> result;
    
    // Flush input buffer first
    tcflush(serial_fd_, TCIFLUSH);
    
    if (!send_command("e")) {
        return result;
    }
    
    std::string response = read_response(200);
    
    // Parse response: "FL FR RL RR"
    std::istringstream iss(response);
    int32_t fl, fr, rl, rr;
    
    if (iss >> fl >> fr >> rl >> rr) {
        result.push_back(fl);
        result.push_back(fr);
        result.push_back(rl);
        result.push_back(rr);
    }
    
    return result;
}

bool DummyBotHardwareInterface::send_motor_speeds(int32_t fl, int32_t fr, int32_t rl, int32_t rr)
{
    std::ostringstream cmd;
    cmd << "m " << fl << " " << fr << " " << rl << " " << rr;
    
    if (!send_command(cmd.str())) {
        return false;
    }
    
    // Optional: read "OK" response
    std::string response = read_response(100);
    return (response.find("OK") != std::string::npos);
}

bool DummyBotHardwareInterface::load_calibration_from_esp32()
{
    // For now, we'll use default calibration factors (1.0)
    // ESP32 stores calibration in EEPROM but doesn't expose a read command yet
    // This can be extended later with a new ESP32 command like "g" (get calibration)
    
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
               "Using default calibration factors (1.0 for all motors)");
    
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
        joint_configs_[i].calibration_factor = 1.0;
    }
    
    return true;
}

// ============================================================================
// CONVERSION HELPERS
// ============================================================================

double DummyBotHardwareInterface::encoder_ticks_to_radians(int32_t ticks)
{
    // radians = (ticks / ticks_per_rev) * 2π
    return (static_cast<double>(ticks) / ticks_per_revolution_) * 2.0 * M_PI;
}

int32_t DummyBotHardwareInterface::radians_per_sec_to_ticks_per_frame(double rad_per_sec)
{
    // ticks_per_frame = (rad/s * ticks_per_rev) / (2π * PID_rate)
    // PID rate is 30Hz (from ESP32 config)
    const double PID_RATE = 30.0;
    
    double ticks_per_sec = (rad_per_sec * ticks_per_revolution_) / (2.0 * M_PI);
    int32_t ticks_per_frame = static_cast<int32_t>(ticks_per_sec / PID_RATE);
    
    return ticks_per_frame;
}

}  // namespace dummybot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    dummybot_hardware::DummyBotHardwareInterface, 
    hardware_interface::SystemInterface)