#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2_ros/transform_broadcaster.h"

using namespace std;

class IMUDisplacementTransformBroadcaster : public rclcpp::Node {
public:
  IMUDisplacementTransformBroadcaster() : Node("imu_displacement_transform_broadcaster") {
    // Subscribe to the IMU topic.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      10,
      std::bind(&IMUDisplacementTransformBroadcaster::ImuCallback, this, std::placeholders::_1));

    // Create a publisher for the displacement topic.
    displacement_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/displacement",
      10);

    // Create a TF broadcaster.
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize the displacement.
    displacement_.linear.x = 0.0f;
    displacement_.linear.y = 0.0f;
    displacement_.linear.z = 0.0f;
  }

private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Get the linear acceleration from the IMU message.
    float acceleration[3];
    acceleration[0] = msg->linear_acceleration.x;
    acceleration[1] = msg->linear_acceleration.y;
    acceleration[2] = msg->linear_acceleration.z;

    // Integrate the linear acceleration twice to get the displacement.
    displacement_.linear.x += 0.5 * acceleration[0] * msg->header.stamp.sec* msg->header.stamp.sec;
    displacement_.linear.y += 0.5 * acceleration[1] * msg->header.stamp.sec* msg->header.stamp.sec;
    displacement_.linear.z += 0.5 * acceleration[2] * msg->header.stamp.sec* msg->header.stamp.sec;

    // Publish the displacement to the displacement topic.
    displacement_pub_->publish(displacement_);

    // Broadcast the transform from the IMU frame to the displacement frame.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "imu";
    transform.child_frame_id = "displacement";
    transform.transform.translation.x = displacement_.linear.x;
    transform.transform.translation.y = displacement_.linear.y;
    transform.transform.translation.z = displacement_.linear.z;
    transform.transform.rotation.w = 1.0f;

    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr displacement_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Twist displacement_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IMUDisplacementTransformBroadcaster>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}