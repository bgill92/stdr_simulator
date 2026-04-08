#include "stdr_server/stdr_server_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stdr_server::StdrServerNode>());
  rclcpp::shutdown();
  return 0;
}
