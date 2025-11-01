#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

using std::string;

class NodeSubscribe : public rclcpp::Node
{
	private:
		string _name;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscribe_;
		
		void sub_callback(const std_msgs::msg::String msg){
			RCLCPP_INFO(this->get_logger(), "receive message: %s", msg.data.c_str());
		}

	public:
		NodeSubscribe(const string & name) : Node(name) {
			_name = name;
			RCLCPP_INFO(this->get_logger(), "%s", _name.c_str());
			command_subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&NodeSubscribe::sub_callback, this, std::placeholders::_1));
				
		}	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<NodeSubscribe>("topic_subscriber_01");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

