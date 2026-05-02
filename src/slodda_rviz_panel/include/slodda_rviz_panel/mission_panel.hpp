#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QPushButton>

namespace slodda_rviz_panel
{

class MissionPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MissionPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onStart();
  void onApprove();
  void onReturnHome();
  void onAbort();

private:
  void publish(const std::string & cmd);
  void applyState(const std::string & state);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  QLabel      * status_label_{nullptr};
  QPushButton * btn_start_{nullptr};
  QPushButton * btn_approve_{nullptr};
  QPushButton * btn_return_{nullptr};
  QPushButton * btn_abort_{nullptr};
};

}  // namespace slodda_rviz_panel
