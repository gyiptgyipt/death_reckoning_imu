#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class IMU_DR : public rclcpp::Node {
public:
    IMU_DR() : Node("ball_tracker") {

        
        twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "imu_twist", 10, std::bind(&IMU_DR::PoseCallback, this, std::placeholders::_1));
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        


        // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        // timer_ = this->create_wall_timer(
        // 100ms, std::bind(&IMU_DR::imuCallback, this));



    }

private:
    // void rec_data(const sensor_msgs::msg::Imu::SharedPtr msg) {
    
    // }

    void PoseCallback( const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
       
        // Extract orientation from the IMU message
        // tf2::Quaternion tf_orientation;
        // tf2::fromMsg(msg->orientation, tf_orientation);

        // Create a transformation message
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time when = this->get_clock()->now() - rclcpp::Duration(5, 0);
        rclcpp::Time t_delta = now + rclcpp::Duration(1/10, 0);

        double time_diff = (now-when).seconds();

        geometry_msgs::msg::TransformStamped transform_stamped;
        geometry_msgs::msg::Pose pose;  

        double pos_x;
        auto dis_x = (msg->linear.x+0.03) * time_diff ;
        pos_x += dis_x;
        // double pos_y = msg->linear_acceleration.y ;
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
        pose.position.x = pos_x;
        // auto n_pos_x =+  pos_x*0.01;
        // auto n_pos_y =+  pos_y*0.01;
    

        
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "odom";  // Parent frame
        transform_stamped.child_frame_id = "imu_link";   // Child frame
        transform_stamped.transform.translation.x = pos_x;  
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0; //msg->linear_acceleration.z;

        // tf2::Quaternion q;
        // q.setRPY(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        // transform_stamped.transform.rotation.x = q.x();
        // transform_stamped.transform.rotation.y = q.y();
        // transform_stamped.transform.rotation.z = q.z();
        // transform_stamped.transform.rotation.w = q.w();
      

        
        // transform_stamped.transform.rotation = tf2::toMsg(tf_orientation);

        // Broadcast the transformation
        tf_broadcaster_->sendTransform(transform_stamped);

    }
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMU_DR>());
    rclcpp::shutdown();
    return 0;
}