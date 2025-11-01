#include "rclcpp/rclcpp.hpp"
#include "example_interfaces_my/srv/echo.hpp"
#include <string>

using std::string;


class ServiceClient01 : public rclcpp::Node {
public:
  ServiceClient01(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    client_ = this->create_client<example_interfaces_my::srv::Echo>("ECHO");
  }
  void send_request(string text) {
    RCLCPP_INFO(this->get_logger(), "向server发送文本 %s", text.c_str());

    while(!client_->wait_for_service(std::chrono::seconds(1))){
      if (!rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务上线中");
    }
    
    // 构造请求
    auto request = std::make_shared<example_interfaces_my::srv::Echo_Request>();
    request->push = text;

    client_->async_send_request(request, std::bind(&ServiceClient01::result_callback_, this, std::placeholders::_1));

  }

private:
  rclcpp::Client<example_interfaces_my::srv::Echo>::SharedPtr client_;

  void result_callback_(rclcpp::Client<example_interfaces_my::srv::Echo>::SharedFuture result_future)
  {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "收到服务器的回复: %s", response->back.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceClient01>("service_client_01");
  node->send_request("1145141919810");
  // rclcpp::sleep_for(std::chrono::seconds(1));
  node->send_request("这是何意味");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

