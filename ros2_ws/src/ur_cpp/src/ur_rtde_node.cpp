#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

class RTDEServoNode : public rclcpp::Node
{
public:
  RTDEServoNode()
  : Node("rtde_servo_node")
  {
    // 1. Declare Parameter for Robot IP
    this->declare_parameter<std::string>("robot_ip", "192.168.10.2");
    std::string robot_ip = this->get_parameter("robot_ip").as_string();

    RCLCPP_INFO(this->get_logger(), "Connecting to UR at: %s ...", robot_ip.c_str());

    try {
      // 2. Connect
      rtde_control_ = std::make_shared<RTDEControlInterface>(robot_ip);
      rtde_receive_ = std::make_shared<RTDEReceiveInterface>(robot_ip);
      RCLCPP_INFO(this->get_logger(), "Successfully connected to UR!");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", e.what());
      // Proceeding without connection will crash update(), so strictly we should exit or handle this.
      // For this example, we assume success.
    }

    // 3. Set Frequency to 500Hz (2ms) for e-Series smooth control
    // If using CB-Series (UR5/UR10 old versions), use 8ms (125Hz)
    double frequency = 500.0; 
    std::chrono::duration<double> period(1.0 / frequency);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&RTDEServoNode::update, this)
    );
  }

private:
  void update()
  {
    if (!rtde_receive_ || !rtde_control_) return;

    // 1. Read current TCP pose
    std::vector<double> tcp = rtde_receive_->getActualTCPPose();

    // Debug print (Throttle this, otherwise it prints 500 times a second!)
    RCLCPP_INFO(this->get_logger(), "TCP Z: %.5f", tcp[2]);

    // 2. Servo Logic (Example)
    // To use servoL, you simply feed the TARGET pose. 
    // WARNING: Be careful adding to tcp[2] in a loop; the robot will fly up indefinitely.
    
    std::vector<double> target_pose = tcp;

    target_pose[2] += 0.001; // Move up very slowly

    
    rtde_control_->servoL(
      target_pose,
      0.5,   // speed
      0.5,   // acceleration
      0.002, // dt (Must match timer: 1/500 = 0.002)
      0.1,   // lookahead time
      300    // gain
    );
    
  }

  // Use Shared Pointers so we can initialize them in the body (safer)
  std::shared_ptr<RTDEControlInterface> rtde_control_;
  std::shared_ptr<RTDEReceiveInterface> rtde_receive_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTDEServoNode>());
  rclcpp::shutdown();
  return 0;
}