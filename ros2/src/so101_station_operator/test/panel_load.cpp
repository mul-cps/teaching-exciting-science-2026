// No hardware or follower services; plugin factory/Qt constructor check only.
#include <QApplication>
#include <QPushButton>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <iostream>
int main(int argc, char ** argv) {
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "75") return 2;
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  {
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto panel = loader.createSharedInstance("so101_station_operator/Operations");
    panel->resize(360, 420);
    panel->show();
    app.processEvents();
    const auto buttons = panel->findChildren<QPushButton *>();
    if (buttons.size() != 17 || panel->width() > 360 || panel->height() > 420) return 3;
    int previous_bottom = -1;
    for (const auto & label : {"Restart entire station", "Restart MoveIt", "ZERO TORQUE / STOP"}) {
      QPushButton * found = nullptr;
      for (auto button : buttons) if (button->text() == label) found = button;
      if (!found || !found->isVisible() || found->width() < panel->width()-30 ||
          found->height() < 34 || found->y() <= previous_bottom) return 4;
      previous_bottom = found->geometry().bottom();
      std::cout << label << ": " << found->x() << "," << found->y() << " " << found->width() << "x" << found->height() << std::endl;
    }
    app.processEvents();
    std::cout << "Operations plugin loaded, constructed, 17 buttons present; restart controls visible at 360x420 px panel size" << std::endl;
  }
  rclcpp::shutdown();
}
