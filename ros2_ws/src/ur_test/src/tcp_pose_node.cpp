#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TcpPoseNode : public rclcpp::Node
{
public:
    TcpPoseNode()
        : Node("tcp_pose_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), std::bind(&TcpPoseNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        try
        {
            // Look up the transform from the robot base to the tool0 (TCP) frame
            auto transform = tf_buffer_.lookupTransform("base", "tool0", tf2::TimePointZero);
            // Translation
            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;
            double z = transform.transform.translation.z;

            // Rotation (Quaternion)
            double qx = transform.transform.rotation.x;
            double qy = transform.transform.rotation.y;
            double qz = transform.transform.rotation.z;
            double qw = transform.transform.rotation.w;

            RCLCPP_INFO(this->get_logger(), "TCP Pose: Translation [%.5f, %.5f, %.5f], Rotation [%.5f, %.5f, %.5f, %.5f]",
                        x, y, z, qx, qy, qz, qw);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get TCP pose: %s", ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpPoseNode>());
    rclcpp::shutdown();
    return 0;
}
