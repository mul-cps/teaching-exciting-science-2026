#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <vector>

namespace so101_station_operator {
class OperatorPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit OperatorPanel(QWidget * parent = nullptr);
  ~OperatorPanel() override;
private:
  void request(const QString & command);
  void tick();
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
  std::vector<QPushButton *> buttons_;
  QLabel * status_;
  QLabel * result_;
  QTimer timer_;
  QElapsedTimer received_, requested_;
  bool pending_ = false;
  bool fresh_ = false;
  bool uncertain_ = false;
  bool damping_supported_ = false;
};
}
