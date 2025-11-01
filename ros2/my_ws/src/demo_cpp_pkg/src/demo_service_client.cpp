#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClient01 : public rclcpp::Node {
public:
  ServiceClient01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_srv");
  }
  void send_request(int a, int b) {
    RCLCPP_INFO(this->get_logger(), "计算 %d+%d", a, b);

    while(!client_->wait_for_service(std::chrono::seconds(1))){
      if (!rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务上线中");
    }
    
    // 构造请求
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
    request->a = a;
    request->b = b;

    client_->async_send_request(request, std::bind(&ServiceClient01::result_callback_, this, std::placeholders::_1));

  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

  void result_callback_(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future)
  {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "计算结果: %ld", response->sum);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceClient01>("service_client_01");
  node->send_request(5, 6);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

