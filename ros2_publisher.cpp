#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

float body_x = 0, body_y = 1, body_z = 2, arms = 3, track_left = 4, track_right = 5;

class FloatPublisher : public rclcpp::Node{
public:
    FloatPublisher() : Node("float_publisher"){
        publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("float_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&FloatPublisher::publish_floats, this));
    }
private:
    void publish_floats(){
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = { body_x, body_y, body_z, arms, track_left, track_right };
        RCLCPP_INFO(this->get_logger(), "Publishing...");
        publisher->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatPublisher>());
    rclcpp::shutdown();
    return 0;
}
