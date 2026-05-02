#include "slodda_rviz_panel/mission_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <QVBoxLayout>
#include <QFrame>
#include <QFont>
#include <QMetaObject>
#include <QString>

namespace slodda_rviz_panel
{

// ── State → enabled button map ────────────────────────────────────────────────
static bool stateEnables(const std::string & state, const std::string & btn)
{
  if (btn == "START")       return state == "IDLE";
  if (btn == "APPROVE")     return state == "AT_BEAR";
  if (btn == "RETURN_HOME") return state == "NAVIGATE" || state == "CANCELING" ||
                                   state == "TRACK_BEAR" || state == "AT_BEAR";
  if (btn == "ABORT")       return state == "NAVIGATE" || state == "CANCELING" ||
                                   state == "TRACK_BEAR";
  return false;
}

static QString stateColor(const std::string & state)
{
  if (state == "IDLE")        return "#aaaaaa";
  if (state == "NAVIGATE")    return "#4a9fe8";
  if (state == "CANCELING")   return "#4a9fe8";
  if (state == "TRACK_BEAR")  return "#e8a020";
  if (state == "AT_BEAR")     return "#40c040";
  if (state == "RETURN_HOME") return "#c060c0";
  return "#aaaaaa";
}

// ── Constructor — build UI only (no ROS node yet) ────────────────────────────
MissionPanel::MissionPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;
  layout->setSpacing(6);
  layout->setContentsMargins(10, 10, 10, 10);

  status_label_ = new QLabel("Status: ---");
  status_label_->setAlignment(Qt::AlignCenter);
  QFont f = status_label_->font();
  f.setPointSize(10);
  f.setBold(true);
  status_label_->setFont(f);
  layout->addWidget(status_label_);

  auto * sep = new QFrame;
  sep->setFrameShape(QFrame::HLine);
  sep->setFrameShadow(QFrame::Sunken);
  layout->addWidget(sep);

  btn_start_   = new QPushButton(QString::fromUtf8("▶  Start Mission"));
  btn_approve_ = new QPushButton(QString::fromUtf8("✓  Approve at Bear"));
  btn_return_  = new QPushButton(QString::fromUtf8("⟵  Return Home"));
  btn_abort_   = new QPushButton(QString::fromUtf8("✕  Abort"));

  for (auto * btn : {btn_start_, btn_approve_, btn_return_, btn_abort_}) {
    btn->setMinimumHeight(40);
    btn->setEnabled(false);
    layout->addWidget(btn);
  }

  setLayout(layout);

  connect(btn_start_,   &QPushButton::clicked, this, &MissionPanel::onStart);
  connect(btn_approve_, &QPushButton::clicked, this, &MissionPanel::onApprove);
  connect(btn_return_,  &QPushButton::clicked, this, &MissionPanel::onReturnHome);
  connect(btn_abort_,   &QPushButton::clicked, this, &MissionPanel::onAbort);
}

// ── onInitialize — ROS node is ready here ────────────────────────────────────
void MissionPanel::onInitialize()
{
  auto node = getDisplayContext()
    ->getRosNodeAbstraction().lock()->get_raw_node();

  pub_ = node->create_publisher<std_msgs::msg::String>("/mission/cmd", 10);

  sub_ = node->create_subscription<std_msgs::msg::String>(
    "/mission/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const std::string state = msg->data;
      // Must update Qt widgets from the GUI thread
      QMetaObject::invokeMethod(this, [this, state]() {
        applyState(state);
      }, Qt::QueuedConnection);
    });

  // Show IDLE immediately
  applyState("IDLE");
}

// ── Apply state to labels and button enable/disable ──────────────────────────
void MissionPanel::applyState(const std::string & state)
{
  const QString color = stateColor(state);
  status_label_->setText(
    QString("Status: <span style='color:%1'>%2</span>")
      .arg(color)
      .arg(QString::fromStdString(state)));
  status_label_->setTextFormat(Qt::RichText);

  btn_start_->setEnabled(stateEnables(state, "START"));
  btn_approve_->setEnabled(stateEnables(state, "APPROVE"));
  btn_return_->setEnabled(stateEnables(state, "RETURN_HOME"));
  btn_abort_->setEnabled(stateEnables(state, "ABORT"));
}

// ── Button slots ──────────────────────────────────────────────────────────────
void MissionPanel::publish(const std::string & cmd)
{
  std_msgs::msg::String msg;
  msg.data = cmd;
  pub_->publish(msg);
}

void MissionPanel::onStart()      { publish("START"); }
void MissionPanel::onApprove()    { publish("APPROVE"); }
void MissionPanel::onReturnHome() { publish("RETURN_HOME"); }
void MissionPanel::onAbort()      { publish("ABORT"); }

}  // namespace slodda_rviz_panel

PLUGINLIB_EXPORT_CLASS(slodda_rviz_panel::MissionPanel, rviz_common::Panel)
