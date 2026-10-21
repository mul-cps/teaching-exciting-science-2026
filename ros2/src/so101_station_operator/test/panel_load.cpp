// No hardware or follower services; plugin factory/Qt constructor check only.
#include <QApplication>
#include <QPushButton>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <iostream>
int main(int argc, char ** argv) {
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "87") return 2;
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  {
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto panel = loader.createSharedInstance("so101_station_operator/Operations");
    if (panel->findChildren<QPushButton *>().size() != 13) return 3;
    app.processEvents();
    std::cout << "Operations plugin loaded, constructed, 13 buttons present" << std::endl;
  }
  rclcpp::shutdown();
}
