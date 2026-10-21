// Isolated ROS domain: no services or hardware. Construction must not actuate.
#include <QApplication>
#include <QPushButton>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <QThread>
#include <iostream>
#include <QMessageBox>
#include <QTimer>
#include <so101_station_workflow/srv/confirm_reference.hpp>
int main(int argc, char ** argv) {
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "75") return 2;
  rclcpp::init(argc, argv); QApplication app(argc, argv);
  {
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto panel = loader.createSharedInstance("so101_station_operator/EV3Operations");
    auto buttons = panel->findChildren<QPushButton *>();
    if (buttons.size() != 10) return 3;
    for (auto button : buttons) {
      if (button->isEnabled() != button->text().startsWith("STOP")) return 4;
    }
    auto node = std::make_shared<rclcpp::Node>("ev3_panel_mock");
    auto publisher = node->create_publisher<std_msgs::msg::String>("/station/ev3/operator_status", 1);
    int calls = 0;
    auto service = node->create_service<std_srvs::srv::Trigger>("/station/stop",
      [&calls](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        ++calls; response->success = true; response->message = "mock stop acknowledged";
      });
    auto pump = [&] { for (int i=0; i<20; ++i) { app.processEvents(); rclcpp::spin_some(node); QThread::msleep(10); } };
    pump();
    if (calls != 0) return 5;
    std_msgs::msg::String status;
    status.data = R"({"schema":1,"connected":true,"fresh":false,"busy":true,"capabilities":{"home":{"enabled":false}}})";
    publisher->publish(status); pump();
    QPushButton * stop = nullptr;
    for (auto button : buttons) {
      if (button->text().startsWith("STOP")) stop = button;
      else if (button->isEnabled()) return 6;
    }
    if (!stop || !stop->isEnabled()) return 7;
    stop->click(); pump();
    if (calls != 1) return 8;
    int confirmations = 0;
    bool correct_request = false;
    auto reference_service = node->create_service<so101_station_workflow::srv::ConfirmReference>(
      "/station/ev3/confirm_reference",
      [&](const so101_station_workflow::srv::ConfirmReference::Request::SharedPtr request,
          so101_station_workflow::srv::ConfirmReference::Response::SharedPtr response) {
        ++confirmations;
        correct_request = request->name == "feeder" && request->reference == "eject" && request->physically_verified;
        response->success = true; response->message = "physical reference recorded";
      });
    pump();
    status.data = R"({"schema":1,"connected":true,"fresh":true,"busy":false,"references":{"feeder":null},"capabilities":{"confirm_feeder":{"enabled":true}}})";
    publisher->publish(status); pump();
    QPushButton * reference = nullptr;
    for (auto button : buttons) if (button->text().startsWith("Record feeder")) reference = button;
    if (!reference || !reference->isEnabled()) return 9;
    auto answer = [&](QMessageBox::StandardButton choice) {
      QTimer::singleShot(0, [choice] {
        auto dialog = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
        if (dialog && dialog->button(choice)) dialog->button(choice)->click();
      });
      reference->click(); pump();
    };
    answer(QMessageBox::No);
    if (confirmations != 0) return 10;
    answer(QMessageBox::Yes);
    if (confirmations != 1 || !correct_request) { std::cerr << "confirmations=" << confirmations << " correct=" << correct_request << std::endl; return 11; }
    std::cout << "EV3 plugin: ten actions; zero startup calls; busy/stale STOP; reference No sends zero; Yes sends exact typed physical acknowledgement\n";
  }
  rclcpp::shutdown();
}
