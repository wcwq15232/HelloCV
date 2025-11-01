#include "rclcpp/rclcpp.hpp"
#include "example_interfaces_my/srv/echo.hpp"
#include <string>

using std::string;

class ServiceServer01 : public rclcpp::Node
{
public:
  ServiceServer01(string name) : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    echo_server_ = this->create_service<example_interfaces_my::srv::Echo>(
        "ECHO",
        std::bind(&ServiceServer01::handle_add_echo, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<example_interfaces_my::srv::Echo>::SharedPtr echo_server_;

  void handle_add_echo(
      const std::shared_ptr<example_interfaces_my::srv::Echo::Request> request,
      std::shared_ptr<example_interfaces_my::srv::Echo::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "收到字符串 %s", request->push.c_str());
    response->back = "已收到你的消息 " + request->push;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceServer01>("server_echo");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
