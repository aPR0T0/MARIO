#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class listener : public rclcpp::Node
{
    private:
        
        void topic_callback(const std_msgs::msg::String & msg) 
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        
        }
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    public:
        
        listener()
        : Node("minimal_subscriber")
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&listener::topic_callback, this, _1));
        
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<listener>());
    rclcpp::shutdown();
    
    return 0;
}