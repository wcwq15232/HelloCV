#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
using std::string;

class NodePublisher : public rclcpp:: Node
{
	private:
		string _name;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		
		void timer_callback(){
			std_msgs::msg::String message;
			message.data = "啊哈哈...";
			RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
			command_publisher_->publish(message);
		}

	public:
		NodePublisher(const string & name):Node(name){
			_name = name;
			RCLCPP_INFO(this->get_logger(), "%s", this->_name.c_str());
			command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
			timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NodePublisher::timer_callback, this));
		}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodePublisher>("topic_publisher_01");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}