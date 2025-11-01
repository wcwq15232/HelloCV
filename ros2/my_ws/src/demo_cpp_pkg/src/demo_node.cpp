#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>

using std::string;

class DemoNode : public rclcpp::Node
{
	private:
		string _name;
	public:
		DemoNode(const string & node_name):Node(node_name)
		{
			this->_name = node_name;
			RCLCPP_INFO(this->get_logger(), "node: %s created", node_name.c_str());
		}

		void out(const string& text){
			RCLCPP_INFO(this->get_logger(), "%s: %s", this->_name.c_str(), text.c_str());
		}
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DemoNode>("demo_node");
	node->out("愿此行 终抵群星");
	node->out("啊哈");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

