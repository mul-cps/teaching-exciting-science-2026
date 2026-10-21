#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <so101_station_workflow/srv/confirm_reference.hpp>
#include <optional>
#include <QElapsedTimer>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <map>
namespace so101_station_operator {
class EV3Panel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit EV3Panel(QWidget * parent = nullptr);
  ~EV3Panel() override;
private:
  void request(const QString & command);
  void tick();
  void receive(const std::string & data);
  void acknowledge(const QString & command, bool success, const QString & message);
  struct Pending {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future;
    QElapsedTimer elapsed;
    int64_t request_id;
  };
  struct ReferencePending {
    rclcpp::Client<so101_station_workflow::srv::ConfirmReference>::SharedPtr client;
    rclcpp::Client<so101_station_workflow::srv::ConfirmReference>::SharedFuture future;
    QElapsedTimer elapsed;
    int64_t request_id;
  };
  std::optional<ReferencePending> reference_pending_;
  rclcpp::Client<so101_station_workflow::srv::ConfirmReference>::SharedPtr reference_client_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::map<QString, Pending> pending_;
  std::map<QString, QPushButton *> buttons_;
  QJsonObject capabilities_;
  QLabel * status_;
  QLabel * result_;
  QLabel * setup_;
  QTimer timer_;
  QElapsedTimer received_;
  QElapsedTimer job_elapsed_;
  QString active_job_;
  QString active_command_;
  bool uncertain_ = false;
};
}
