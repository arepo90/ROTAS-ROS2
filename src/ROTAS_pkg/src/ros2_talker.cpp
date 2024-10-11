#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

float a = 0.0, b = 0.0, c = 0.0;

class FloatPublisher : public rclcpp::Node{
public:
    FloatPublisher() : Node("float_publisher"){
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("float_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
            std::bind(&FloatPublisher::publish_floats, this));
    }
private:
    void publish_floats(){
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = {a, b, c};

        // --- test data ---
        a += 1.5;
        b += 3;
        c += 5.4;
        
        RCLCPP_INFO(this->get_logger(), "Publishing: [%.2f, %.2f, %.2f]", message.data[0], message.data[1], message.data[2]);
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatPublisher>());
    rclcpp::shutdown();
    return 0;
}