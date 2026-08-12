#include "sim2real_ground_station/interactive_goal_panel.hpp"

#include <cmath>

#include <boost/bind/bind.hpp>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
#include <rviz/config.h>

namespace sim2real_ground_station {
namespace {

constexpr double kStateTimeoutSeconds = 3.0;
constexpr char kActionName[] = "/ground_station/interactive_goal";
constexpr char kCandidateTopic[] = "/ground_station/goal_candidate";
constexpr double kRadiansToDegrees = 57.29577951308232;

double yawDegrees(const geometry_msgs::Quaternion& q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-6) {
    return 0.0;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z)) * kRadiansToDegrees;
}

}  // namespace

InteractiveGoalPanel::InteractiveGoalPanel(QWidget* parent)
    : rviz::Panel(parent), action_client_(new GoalClient(kActionName, true)) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);

  endpoint_label_ = new QLabel(
      tr("固定无人机：192.168.1.123\n地面站：192.168.1.124"), this);
  endpoint_label_->setWordWrap(true);
  root->addWidget(endpoint_label_);

  candidate_label_ = new QLabel(tr("请使用 2D Nav Goal 选择目标"), this);
  candidate_label_->setWordWrap(true);
  root->addWidget(candidate_label_);

  auto* form = new QFormLayout();
  goal_height_ = new QDoubleSpinBox(this);
  goal_height_->setRange(0.5, 2.5);
  goal_height_->setDecimals(2);
  goal_height_->setSingleStep(0.1);
  goal_height_->setValue(1.5);
  goal_height_->setSuffix(tr(" m"));
  form->addRow(tr("目标高度"), goal_height_);

  takeoff_height_ = new QDoubleSpinBox(this);
  takeoff_height_->setRange(0.5, 2.5);
  takeoff_height_->setDecimals(2);
  takeoff_height_->setSingleStep(0.1);
  takeoff_height_->setValue(1.5);
  takeoff_height_->setSuffix(tr(" m"));
  form->addRow(tr("起飞高度"), takeoff_height_);
  root->addLayout(form);

  send_button_ = new QPushButton(tr("发送目标"), this);
  send_button_->setEnabled(false);
  connect(send_button_, &QPushButton::clicked,
          this, &InteractiveGoalPanel::onSendGoal);
  root->addWidget(send_button_);

  status_label_ = new QLabel(tr("正在等待机载控制服务…"), this);
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  root->addStretch(1);

  connect(this, &InteractiveGoalPanel::candidateForUi,
          this, &InteractiveGoalPanel::onCandidateForUi,
          Qt::QueuedConnection);
  connect(this, &InteractiveGoalPanel::statusForUi,
          this, &InteractiveGoalPanel::onStatusForUi,
          Qt::QueuedConnection);
  connect(this, &InteractiveGoalPanel::actionFinishedForUi,
          this, &InteractiveGoalPanel::onActionFinishedForUi,
          Qt::QueuedConnection);

  candidate_subscriber_ = node_.subscribe(
      kCandidateTopic, 1, &InteractiveGoalPanel::candidateCallback, this);
  state_subscriber_ = node_.subscribe(
      "/mavros/state", 1, &InteractiveGoalPanel::stateCallback, this);
  extended_state_subscriber_ = node_.subscribe(
      "/mavros/extended_state", 1,
      &InteractiveGoalPanel::extendedStateCallback, this);

  readiness_timer_ = new QTimer(this);
  connect(readiness_timer_, &QTimer::timeout,
          this, &InteractiveGoalPanel::refreshReadiness);
  readiness_timer_->start(500);
}

void InteractiveGoalPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("Goal Height", goal_height_->value());
  config.mapSetValue("Takeoff Height", takeoff_height_->value());
}

void InteractiveGoalPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  float value = 0.0F;
  if (config.mapGetFloat("Goal Height", &value)) {
    goal_height_->setValue(value);
  }
  if (config.mapGetFloat("Takeoff Height", &value)) {
    takeoff_height_->setValue(value);
  }
}

void InteractiveGoalPanel::candidateCallback(
    const geometry_msgs::PoseStampedConstPtr& message) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    candidate_ = *message;
    candidate_.header.frame_id = "world";
    has_candidate_ = true;
  }
  Q_EMIT candidateForUi(message->pose.position.x, message->pose.position.y,
                        yawDegrees(message->pose.orientation));
}

void InteractiveGoalPanel::stateCallback(
    const mavros_msgs::StateConstPtr& message) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  state_ = *message;
  has_state_ = true;
  state_received_at_ = ros::WallTime::now();
}

void InteractiveGoalPanel::extendedStateCallback(
    const mavros_msgs::ExtendedStateConstPtr& message) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  extended_state_ = *message;
  has_extended_state_ = true;
  extended_state_received_at_ = ros::WallTime::now();
}

bool InteractiveGoalPanel::stateIsFresh(
    const ros::WallTime& received_at) const {
  return !received_at.isZero() &&
         (ros::WallTime::now() - received_at).toSec() <= kStateTimeoutSeconds;
}

void InteractiveGoalPanel::onCandidateForUi(
    double x, double y, double yaw_degrees) {
  candidate_label_->setText(
      tr("候选目标：X=%1  Y=%2  航向=%3°")
          .arg(x, 0, 'f', 2)
          .arg(y, 0, 'f', 2)
          .arg(yaw_degrees, 0, 'f', 1));
  refreshReadiness();
}

void InteractiveGoalPanel::onStatusForUi(
    const QString& text, bool error) {
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "color: #ff6b6b;" : "color: #8bd5ff;");
}

void InteractiveGoalPanel::onActionFinishedForUi(
    const QString& text, bool success) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    action_active_ = false;
  }
  onStatusForUi(text, !success);
  refreshReadiness();
}

void InteractiveGoalPanel::showError(const QString& text) {
  onStatusForUi(text, true);
  QMessageBox::warning(this, tr("目标发送被拒绝"), text);
}

void InteractiveGoalPanel::onSendGoal() {
  geometry_msgs::PoseStamped target;
  mavros_msgs::State state;
  mavros_msgs::ExtendedState extended_state;
  ros::WallTime state_at;
  ros::WallTime extended_at;
  bool has_candidate = false;
  bool has_state = false;
  bool has_extended_state = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target = candidate_;
    state = state_;
    extended_state = extended_state_;
    state_at = state_received_at_;
    extended_at = extended_state_received_at_;
    has_candidate = has_candidate_;
    has_state = has_state_;
    has_extended_state = has_extended_state_;
  }

  if (!has_candidate) {
    showError(tr("请先使用 2D Nav Goal 选择目标。"));
    return;
  }
  if (!action_client_->isServerConnected()) {
    showError(tr("机载目标服务未连接。"));
    return;
  }
  if (!has_state || !stateIsFresh(state_at) || !state.connected) {
    showError(tr("PX4 连接状态缺失、过期或未连接。"));
    return;
  }

  bool auto_arm = false;
  if (state.armed) {
    if (state.mode != "OFFBOARD") {
      showError(tr("无人机已解锁但不在 OFFBOARD，拒绝自动切换模式。"));
      return;
    }
  } else {
    if (!has_extended_state || !stateIsFresh(extended_at) ||
        extended_state.landed_state !=
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
      showError(tr("无法确认无人机处于地面，禁止自动解锁。"));
      return;
    }
    QMessageBox confirmation(
        QMessageBox::Warning,
        tr("确认自动解锁与起飞"),
        tr("无人机当前未解锁且位于地面。\n"
           "发送后将自动解锁，以 AUTO.TAKEOFF 起飞到 %1 m，"
           "随后进入 OFFBOARD 并飞向目标。\n\n"
           "请确认飞行区域安全并保持遥控器可随时接管。")
            .arg(takeoff_height_->value(), 0, 'f', 2),
        QMessageBox::Ok | QMessageBox::Cancel,
        this);
    confirmation.setDefaultButton(QMessageBox::Cancel);
    confirmation.setEscapeButton(QMessageBox::Cancel);
    if (confirmation.exec() != QMessageBox::Ok) {
      onStatusForUi(tr("操作员已取消自动解锁。"), false);
      return;
    }
    auto_arm = true;
  }

  target.header.stamp = ros::Time::now();
  target.header.frame_id = "world";
  target.pose.position.z = goal_height_->value();
  sim2real_planning_msgs::InteractiveGoalGoal goal;
  goal.target = target;
  goal.takeoff_height = takeoff_height_->value();
  goal.auto_arm_if_grounded = auto_arm;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    action_active_ = true;
  }
  send_button_->setEnabled(false);
  onStatusForUi(tr("正在提交目标…"), false);
  action_client_->sendGoal(
      goal,
      boost::bind(&InteractiveGoalPanel::doneCallback, this,
                  boost::placeholders::_1, boost::placeholders::_2),
      boost::bind(&InteractiveGoalPanel::activeCallback, this),
      boost::bind(&InteractiveGoalPanel::feedbackCallback, this,
                  boost::placeholders::_1));
}

void InteractiveGoalPanel::activeCallback() {
  Q_EMIT statusForUi(tr("机载端已接受目标请求。"), false);
}

void InteractiveGoalPanel::feedbackCallback(
    const sim2real_planning_msgs::InteractiveGoalFeedbackConstPtr& feedback) {
  Q_EMIT statusForUi(QString::fromStdString(feedback->message), false);
}

void InteractiveGoalPanel::doneCallback(
    const actionlib::SimpleClientGoalState& state,
    const sim2real_planning_msgs::InteractiveGoalResultConstPtr& result) {
  const bool success = result && result->success &&
                       state == actionlib::SimpleClientGoalState::SUCCEEDED;
  QString message = result
                        ? QString::fromStdString(result->message)
                        : tr("机载目标服务未返回结果。");
  Q_EMIT actionFinishedForUi(message, success);
}

void InteractiveGoalPanel::refreshReadiness() {
  bool has_candidate = false;
  bool action_active = false;
  bool has_state = false;
  ros::WallTime state_at;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    has_candidate = has_candidate_;
    action_active = action_active_;
    has_state = has_state_;
    state_at = state_received_at_;
  }
  const bool server_ready = action_client_->isServerConnected();
  const bool state_ready = has_state && stateIsFresh(state_at);
  send_button_->setEnabled(
      has_candidate && server_ready && state_ready && !action_active);
  if (!action_active && !server_ready) {
    onStatusForUi(tr("机载目标服务未连接，发送已禁用。"), true);
  } else if (!action_active && !state_ready) {
    onStatusForUi(tr("PX4 状态不可用或已过期，发送已禁用。"), true);
  } else if (!action_active && !has_candidate) {
    onStatusForUi(tr("链路正常，请选择目标。"), false);
  }
}

}  // namespace sim2real_ground_station

PLUGINLIB_EXPORT_CLASS(sim2real_ground_station::InteractiveGoalPanel, rviz::Panel)
