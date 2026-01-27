#include "dummybot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dummybot_hardware
{

// ============================================================================
// INITIALIZATION
// ============================================================================

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Initializing DummyBot Hardware Interface...");

    // Read hardware parameters from URDF
    try {
        serial_port_ = info_.hardware_parameters["serial_port"];
        serial_baud_ = std::stoi(info_.hardware_parameters["serial_baud"]);
        wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
        wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
        ticks_per_revolution_ = std::stoi(info_.hardware_parameters["ticks_per_revolution"]);
        
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Hardware params: port=%s, baud=%d, radius=%.4f, separation=%.4f, tpr=%d",
            serial_port_.c_str(), serial_baud_, wheel_radius_, 
            wheel_separation_, ticks_per_revolution_);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Failed to read hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Read PID parameters from URDF (INT pentru ESP32, ordine: kp kd ki ko)
    try {
        pid_kp_ = std::stoi(info_.hardware_parameters["pid_kp"]);
        pid_kd_ = std::stoi(info_.hardware_parameters["pid_kd"]);
        pid_ki_ = std::stoi(info_.hardware_parameters["pid_ki"]);
        pid_ko_ = std::stoi(info_.hardware_parameters["pid_ko"]);
        
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
            "PID Parameters: Kp=%d, Kd=%d, Ki=%d, Ko=%d",
            pid_kp_, pid_kd_, pid_ki_, pid_ko_);
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("DummyBotHardwareInterface"),
            "PID parameters not found in URDF, using defaults: %s", e.what());
        pid_kp_ = 280;
        pid_kd_ = 70;
        pid_ki_ = 0;
        pid_ko_ = 14;
    }

    // Read Calibration parameters from URDF (FLOAT)
    try {
        calibration_fl_ = std::stod(info_.hardware_parameters["calibration_fl"]);
        calibration_fr_ = std::stod(info_.hardware_parameters["calibration_fr"]);
        calibration_rl_ = std::stod(info_.hardware_parameters["calibration_rl"]);
        calibration_rr_ = std::stod(info_.hardware_parameters["calibration_rr"]);
        
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Calibration: FL=%.3f, FR=%.3f, RL=%.3f, RR=%.3f",
            calibration_fl_, calibration_fr_, calibration_rl_, calibration_rr_);
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Calibration parameters not found in URDF, using defaults: %s", e.what());
        calibration_fl_ = 0.875;
        calibration_fr_ = 1.065;
        calibration_rl_ = 1.025;
        calibration_rr_ = 1.025;
    }

    // Initialize joint configuration
    joint_configs_.clear();
    calibration_factors_.resize(4, 1.0);
    
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        JointConfig config;
        config.name = joint.name;
        
        // Determine encoder index based on joint name
        if (joint.name.find("front_left") != std::string::npos) {
            config.encoder_index = 0;  // FL
            config.calibration_factor = calibration_fl_;
        } else if (joint.name.find("front_right") != std::string::npos) {
            config.encoder_index = 1;  // FR
            config.calibration_factor = calibration_fr_;
        } else if (joint.name.find("rear_left") != std::string::npos) {
            config.encoder_index = 2;  // RL
            config.calibration_factor = calibration_rl_;
        } else if (joint.name.find("rear_right") != std::string::npos) {
            config.encoder_index = 3;  // RR
            config.calibration_factor = calibration_rr_;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Unknown joint name: %s", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        joint_configs_.push_back(config);
        calibration_factors_[config.encoder_index] = config.calibration_factor;
        
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Joint configured: %s -> Encoder %d, Calibration %.3f", 
            config.name.c_str(), config.encoder_index, config.calibration_factor);
    }

    // Resize state/command vectors
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_efforts_.resize(info_.joints.size(), 0.0);
    last_encoder_counts_.resize(4, 0);

    serial_fd_ = -1;
    first_read_ = true;

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Hardware interface initialized successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Configuring hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// INTERFACE EXPORT
// ============================================================================

std::vector<hardware_interface::StateInterface> 
DummyBotHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
DummyBotHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    
    return command_interfaces;
}

// ============================================================================
// ACTIVATION / DEACTIVATION
// ============================================================================

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Activating hardware interface...");

    // Initialize serial
    serial_fd_ = init_serial(serial_port_, serial_baud_);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                     "Failed to open serial port");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Wait for ESP32 to be ready
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Waiting for ESP32 to be ready...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 1. Update PID parameters on ESP32
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Updating PID parameters...");
    if (!update_pid_parameters(pid_kp_, pid_kd_, pid_ki_, pid_ko_)) {
        RCLCPP_WARN(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Failed to update PID parameters");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "PID parameters updated successfully");
    }

    // Small delay between commands
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2. Send calibration to ESP32
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Sending calibration to ESP32...");
    if (!send_calibration(calibration_fl_, calibration_fr_, calibration_rl_, calibration_rr_)) {
        RCLCPP_WARN(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Failed to send calibration");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Calibration sent successfully");
    }

    // Small delay before reset
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. Reset encoders
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Resetting encoders...");
    if (!reset_encoders()) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                     "Failed to reset encoders");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Wait for ESP32 to be ready after reset
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // TEST - Citește encoderele o dată pentru a verifica comunicarea
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Testing encoder communication...");
    auto test_counts = read_encoders();
    if (test_counts.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
                     "Failed to read encoders during activation");
        return hardware_interface::CallbackReturn::ERROR;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                    "Encoder test OK: FL=%d FR=%d RL=%d RR=%d", 
                    test_counts[0], test_counts[1], test_counts[2], test_counts[3]);
    }

    // Initialize state with STEADY_TIME to match controller_manager
    first_read_ = true;
    last_read_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Hardware interface activated successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyBotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"), 
                "Deactivating hardware interface...");

    // Stop motors
    send_motor_speeds(0, 0, 0, 0);
    
    // Close serial
    close_serial();

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
    
    if (encoder_counts.empty()) {
        static auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DummyBotHardwareInterface"),
                             *clock, 1000, "Failed to read encoders");
        return hardware_interface::return_type::ERROR;
    }

    if (first_read_) {
        // First read: just store values
        last_encoder_counts_ = encoder_counts;
        last_read_time_ = time;
        first_read_ = false;
        return hardware_interface::return_type::OK;
    }

    // Calculate time delta
    rclcpp::Duration dt = time - last_read_time_;
    double dt_seconds = dt.seconds();

    if (dt_seconds <= 0.0) {
        // Invalid time delta, skip this cycle
        return hardware_interface::return_type::OK;
    }

    // Update positions and velocities for each joint
    for (size_t i = 0; i < joint_configs_.size(); i++)
    {
        int encoder_idx = joint_configs_[i].encoder_index;
        double calib_factor = joint_configs_[i].calibration_factor;
        
        // Calculate delta ticks
        int32_t delta_ticks = encoder_counts[encoder_idx] - last_encoder_counts_[encoder_idx];
        
        // Convert to radians (with calibration)
        double delta_position = encoder_ticks_to_radians(delta_ticks) * calib_factor;
        
        // Update position (integrate)
        hw_positions_[i] += delta_position;
        
        // Calculate velocity
        hw_velocities_[i] = delta_position / dt_seconds;
    }

    // Store current values for next iteration
    last_encoder_counts_ = encoder_counts;
    last_read_time_ = time;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyBotHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Convert commanded velocities (rad/s) to motor speeds (ticks/frame)
    std::vector<int32_t> motor_speeds(4, 0);
    
    for (size_t i = 0; i < joint_configs_.size(); i++)
    {
        int encoder_idx = joint_configs_[i].encoder_index;
        double calib_factor = joint_configs_[i].calibration_factor;
        
        // Apply calibration factor to command
        double calibrated_velocity = hw_commands_[i] / calib_factor;
        
        // Convert to ticks per frame
        motor_speeds[encoder_idx] = radians_per_sec_to_ticks_per_frame(calibrated_velocity);
    }

    // Send to ESP32
    if (!send_motor_speeds(motor_speeds[0], motor_speeds[1], 
                           motor_speeds[2], motor_speeds[3]))
    {
        static auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DummyBotHardwareInterface"),
                             *clock, 1000, "Failed to send motor speeds");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

int DummyBotHardwareInterface::init_serial(const std::string& port_name, int baud_rate)
{
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Failed to open serial port: %s", port_name.c_str());
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Error getting serial attributes");
        ::close(fd);
        return -1;
    }

    // Set baud rate
    speed_t speed = B115200;
    if (baud_rate == 9600) speed = B9600;
    else if (baud_rate == 19200) speed = B19200;
    else if (baud_rate == 38400) speed = B38400;
    else if (baud_rate == 57600) speed = B57600;
    else if (baud_rate == 115200) speed = B115200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Error setting serial attributes");
        ::close(fd);
        return -1;
    }

    return fd;
}

void DummyBotHardwareInterface::close_serial()
{
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool DummyBotHardwareInterface::send_command(const std::string& cmd)
{
    if (serial_fd_ < 0) {
        return false;
    }

    ssize_t written = ::write(serial_fd_, cmd.c_str(), cmd.length());
    if (written < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DummyBotHardwareInterface"),
            "Failed to write to serial port");
        return false;
    }

    return true;
}

std::string DummyBotHardwareInterface::read_response(int timeout_ms)
{
    if (serial_fd_ < 0) {
        return "";
    }

    std::string response;
    char buf[256];
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
        
        if (elapsed.count() > timeout_ms) {
            break;
        }

        ssize_t n = ::read(serial_fd_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            response += buf;
            
            // Check if we have a complete line
            if (response.find('\n') != std::string::npos) {
                break;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return response;
}

// ============================================================================
// ESP32 PROTOCOL
// ============================================================================

bool DummyBotHardwareInterface::reset_encoders()
{
    if (!send_command("r\n")) {
        return false;
    }
    
    std::string response = read_response(200);
    
    // Reset local tracking
    std::fill(last_encoder_counts_.begin(), last_encoder_counts_.end(), 0);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    
    return !response.empty();
}

std::vector<int32_t> DummyBotHardwareInterface::read_encoders()
{
    // Flush any old data from serial buffer
    tcflush(serial_fd_, TCIFLUSH);
    
    if (!send_command("e\n")) {
        return {};
    }
    
    std::string response = read_response(500);
    
    if (response.empty()) {
        return {};
    }

    // Parse response: "FL FR RL RR"
    std::vector<int32_t> counts;
    std::istringstream iss(response);
    
    for (int i = 0; i < 4; i++) {
        int32_t count;
        if (iss >> count) {
            counts.push_back(count);
        } else {
            return {};
        }
    }
    
    return counts;
}

bool DummyBotHardwareInterface::send_motor_speeds(int32_t fl, int32_t fr, int32_t rl, int32_t rr)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "m %d %d %d %d\n", fl, fr, rl, rr);
    
    return send_command(cmd);
}

bool DummyBotHardwareInterface::update_pid_parameters(int kp, int kd, int ki, int ko)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "u %d %d %d %d\n", kp, kd, ki, ko);
    
    if (!send_command(cmd)) {
        return false;
    }
    
    std::string response = read_response(500);
    
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "PID Update sent: Kp=%d, Kd=%d, Ki=%d, Ko=%d | Response: %s", 
                kp, kd, ki, ko, response.c_str());
    
    return !response.empty();
}

bool DummyBotHardwareInterface::send_calibration(double fl, double fr, double rl, double rr)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "c %.3f %.3f %.3f %.3f\n", fl, fr, rl, rr);
    
    if (!send_command(cmd)) {
        return false;
    }
    
    std::string response = read_response(500);
    
    RCLCPP_INFO(rclcpp::get_logger("DummyBotHardwareInterface"),
                "Calibration sent: FL=%.3f, FR=%.3f, RL=%.3f, RR=%.3f | Response: %s", 
                fl, fr, rl, rr, response.c_str());
    
    return !response.empty();
}

// ============================================================================
// CONVERSION HELPERS
// ============================================================================

double DummyBotHardwareInterface::encoder_ticks_to_radians(int32_t ticks)
{
    return (static_cast<double>(ticks) / ticks_per_revolution_) * 2.0 * M_PI;
}

int32_t DummyBotHardwareInterface::radians_per_sec_to_ticks_per_frame(double rad_per_sec)
{
    // Assuming 50Hz update rate (0.02s per frame)
    double ticks_per_sec = (rad_per_sec / (2.0 * M_PI)) * ticks_per_revolution_;
    return static_cast<int32_t>(ticks_per_sec * 0.02);
}

}  // namespace dummybot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    dummybot_hardware::DummyBotHardwareInterface,
    hardware_interface::SystemInterface)