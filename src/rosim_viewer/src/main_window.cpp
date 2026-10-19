// Legacy Qt GUI: control panel (import, edge list, follow parameters, speed,
// home/jog) wired to SceneView for the 3D display and RosBridge for ROS I/O.
// The browser UI in rosim_server is the primary interface; this remains for
// desktop use via robot_bringup.launch.py use_viewer:=true.

#include "rosim_viewer/main_window.h"

#include <cmath>
#include <cstdlib>

#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSplitter>
#include <QTime>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rosim_interfaces/action/follow_edge.hpp"
#include "rosim_core/occ_geometry_service.h"

using FollowEdge = rosim_interfaces::action::FollowEdge;

namespace rosim::viewer
{

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
  try
  {
    const auto share = ament_index_cpp::get_package_share_directory("rosim_description");
    sample_step_path_ = QString::fromStdString(share + "/meshes/workpiece_sample.step");
  }
  catch (const std::exception&)
  {
    const char* home = std::getenv("HOME");
    sample_step_path_ = QString::fromStdString(home ? home : "") + "/rosim_ws/sample.step";
  }

  bridge_ = new RosBridge(this);
  robot_ = new RobotAssembly(bridge_->node(), this);

  buildUi();

  // ROS -> GUI wiring (queued automatically: emitters are on the spin thread)
  connect(bridge_, &RosBridge::jointStates, this, &MainWindow::onJointState);
  connect(bridge_, &RosBridge::importFinished, this, &MainWindow::onImportFinished);
  connect(bridge_, &RosBridge::followFeedback, this, &MainWindow::onFollowFeedback);
  connect(bridge_, &RosBridge::followResult, this, &MainWindow::onFollowResult);
  connect(bridge_, &RosBridge::goHomeResult, this, &MainWindow::onGoHomeResult);
  connect(bridge_, &RosBridge::jogResult, this, &MainWindow::onJogResult);

  if (robot_->valid())
  {
    robot_->addToScene(scene_->renderer());
    scene_->render();
    log(QString("robot model loaded (%1 links)").arg(
        robot_->model()->getLinkModels().size()));
  }
  else
  {
    log("WARNING: robot model not loaded — is robot_description set?");
  }

  bridge_->startSpin();

  connect(&render_timer_, &QTimer::timeout, this, [this]() {
    if (render_dirty_)
    {
      render_dirty_ = false;
      scene_->render();
    }
  });
  render_timer_.start(30);

  setWindowTitle("rosim — 6-axis edge-following simulator");
  resize(1400, 860);
}

MainWindow::~MainWindow()
{
  if (bridge_)
    bridge_->stopSpin();
}

void MainWindow::buildUi()
{
  scene_ = new SceneView(this);

  // ---------------- left panel ----------------
  auto panel = new QWidget();
  auto layout = new QVBoxLayout(panel);

  // model tree
  auto tree_group = new QGroupBox("工件 / 边");
  auto tree_layout = new QVBoxLayout(tree_group);
  tree_ = new QTreeWidget();
  tree_->setHeaderLabels({ "模型 / 边", "长度" });
  tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  tree_->setMinimumWidth(280);
  connect(tree_, &QTreeWidget::itemSelectionChanged,
          this, &MainWindow::onTreeSelectionChanged);
  tree_layout->addWidget(tree_);
  layout->addWidget(tree_group);

  // import controls
  auto import_group = new QGroupBox("导入工件 (STEP)");
  auto import_layout = new QGridLayout(import_group);
  place_x_ = new QDoubleSpinBox(); place_y_ = new QDoubleSpinBox(); place_z_ = new QDoubleSpinBox();
  for (auto* spin : { place_x_, place_y_, place_z_ })
  {
    spin->setDecimals(3);
    spin->setSingleStep(0.05);
    spin->setRange(-2.0, 2.0);
  }
  // default placement: GP12 welding sweet spot (r ~0.65-0.81 from base axis),
  // long plate edge along y; z kept low enough that the home-pose wrist
  // clears the part (measured: home link5 dips to z=0.559, part top = z+0.115)
  place_x_->setValue(0.65);
  place_y_->setValue(-0.20);
  place_z_->setValue(0.45);
  place_yaw_ = new QDoubleSpinBox();
  place_yaw_->setDecimals(0);
  place_yaw_->setSingleStep(15.0);
  place_yaw_->setRange(-180.0, 180.0);
  place_yaw_->setSuffix("°");
  place_yaw_->setValue(90.0);
  import_layout->addWidget(new QLabel("X"), 0, 0);
  import_layout->addWidget(place_x_, 0, 1);
  import_layout->addWidget(new QLabel("Y"), 0, 2);
  import_layout->addWidget(place_y_, 0, 3);
  import_layout->addWidget(new QLabel("Z"), 0, 4);
  import_layout->addWidget(place_z_, 0, 5);
  import_layout->addWidget(new QLabel("yaw"), 0, 6);
  import_layout->addWidget(place_yaw_, 0, 7);
  import_btn_ = new QPushButton("导入 STEP ...");
  connect(import_btn_, &QPushButton::clicked, this, &MainWindow::onImportClicked);
  import_layout->addWidget(import_btn_, 1, 0, 1, 8);
  layout->addWidget(import_group);

  // follow parameters
  auto param_group = new QGroupBox("随边参数");
  auto param_layout = new QGridLayout(param_group);
  param_layout->addWidget(new QLabel("姿态模式"), 0, 0);
  orient_combo_ = new QComboBox();
  orient_combo_->addItem("沿边切线 (Tangent)", FollowEdge::Goal::ORIENTATION_TANGENT);
  orient_combo_->addItem("固定姿态·竖直向下 (Fixed)", FollowEdge::Goal::ORIENTATION_FIXED);
  param_layout->addWidget(orient_combo_, 0, 1);

  tilt_spin_ = new QDoubleSpinBox();
  tilt_spin_->setDecimals(0);
  tilt_spin_->setSingleStep(5.0);
  tilt_spin_->setRange(0.0, 60.0);
  tilt_spin_->setValue(40.0);
  tilt_spin_->setSuffix(" °");
  param_layout->addWidget(new QLabel("切线倾角"), 4, 0);
  param_layout->addWidget(tilt_spin_, 4, 1);

  tool_offset_spin_ = new QDoubleSpinBox();
  tool_offset_spin_->setDecimals(3);
  tool_offset_spin_->setSingleStep(0.002);
  tool_offset_spin_->setRange(0.0, 0.10);
  tool_offset_spin_->setValue(0.0);
  tool_offset_spin_->setSuffix(" m");
  param_layout->addWidget(new QLabel("工具偏置"), 1, 0);
  param_layout->addWidget(tool_offset_spin_, 1, 1);

  approach_spin_ = new QDoubleSpinBox();
  approach_spin_->setDecimals(3);
  approach_spin_->setSingleStep(0.01);
  approach_spin_->setRange(0.02, 0.30);
  approach_spin_->setValue(0.08);
  approach_spin_->setSuffix(" m");
  param_layout->addWidget(new QLabel("接近距离"), 2, 0);
  param_layout->addWidget(approach_spin_, 2, 1);

  spacing_spin_ = new QDoubleSpinBox();
  spacing_spin_->setDecimals(3);
  spacing_spin_->setSingleStep(0.001);
  spacing_spin_->setRange(0.001, 0.050);
  spacing_spin_->setValue(0.005);
  spacing_spin_->setSuffix(" m");
  param_layout->addWidget(new QLabel("采样间距"), 3, 0);
  param_layout->addWidget(spacing_spin_, 3, 1);
  layout->addWidget(param_group);

  // speed
  auto speed_group = new QGroupBox("执行速度");
  auto speed_layout = new QVBoxLayout(speed_group);
  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(5, 100);
  speed_slider_->setValue(100);
  speed_label_ = new QLabel("100 %");
  connect(speed_slider_, &QSlider::valueChanged, this, &MainWindow::onSpeedChanged);
  speed_layout->addWidget(speed_slider_);
  speed_layout->addWidget(speed_label_);
  layout->addWidget(speed_group);

  // action buttons
  auto btn_layout = new QHBoxLayout();
  follow_btn_ = new QPushButton("规划并跟随 (Follow Edge)");
  follow_btn_->setEnabled(false);
  stop_btn_ = new QPushButton("停止");
  stop_btn_->setEnabled(false);
  connect(follow_btn_, &QPushButton::clicked, this, &MainWindow::onFollowClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopClicked);
  btn_layout->addWidget(follow_btn_, 1);
  btn_layout->addWidget(stop_btn_);
  layout->addLayout(btn_layout);

  // ---------------- manual control ----------------
  auto manual_group = new QGroupBox("手动控制");
  auto manual_layout = new QVBoxLayout(manual_group);

  home_btn_ = new QPushButton("回 Home（OMPL 规划）");
  connect(home_btn_, &QPushButton::clicked, this, &MainWindow::onHomeClicked);
  manual_layout->addWidget(home_btn_);

  auto step_row = new QHBoxLayout();
  step_row->addWidget(new QLabel("点动步长"));
  jog_step_spin_ = new QDoubleSpinBox();
  jog_step_spin_->setDecimals(1);
  jog_step_spin_->setSingleStep(1.0);
  jog_step_spin_->setRange(1.0, 30.0);
  jog_step_spin_->setValue(3.0);
  jog_step_spin_->setSuffix(" °");
  step_row->addWidget(jog_step_spin_);
  step_row->addStretch(1);
  manual_layout->addLayout(step_row);

  auto jog_grid = new QGridLayout();
  const moveit::core::JointModelGroup* jmg =
      (robot_ && robot_->model()) ? robot_->model()->getJointModelGroup("manipulator") : nullptr;
  if (jmg)
  {
    const auto joint_names = jmg->getActiveJointModelNames();
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      const QString name = QString::fromStdString(joint_names[i]);
      const int row = static_cast<int>(i);
      jog_grid->addWidget(new QLabel(name), row, 0);

      auto value = new QLabel("0.0°");
      value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      jog_value_labels_[name] = value;
      jog_grid->addWidget(value, row, 3);

      for (const int sign : { -1, +1 })
      {
        auto btn = new QPushButton(sign < 0 ? "−" : "+");
        btn->setFixedWidth(36);
        connect(btn, &QPushButton::clicked, this, [this, name, sign]() {
          if (manual_pending_)
            return;
          manual_pending_ = true;
          const double rad = jog_step_spin_->value() * M_PI / 180.0 * sign;
          bridge_->sendJogJoint(name, rad);
        });
        jog_grid->addWidget(btn, row, sign < 0 ? 1 : 2);
      }
    }
  }
  else
  {
    jog_grid->addWidget(new QLabel("模型未加载，点动不可用"), 0, 0);
  }
  manual_layout->addLayout(jog_grid);
  layout->addWidget(manual_group);

  // progress
  auto prog_group = new QGroupBox("任务状态");
  auto prog_layout = new QVBoxLayout(prog_group);
  progress_ = new QProgressBar();
  progress_->setRange(0, 100);
  phase_label_ = new QLabel("idle");
  prog_layout->addWidget(progress_);
  prog_layout->addWidget(phase_label_);
  layout->addWidget(prog_group);

  // log
  auto log_group = new QGroupBox("日志");
  auto log_layout = new QVBoxLayout(log_group);
  log_box_ = new QPlainTextEdit();
  log_box_->setReadOnly(true);
  log_box_->setMaximumBlockCount(500);
  log_layout->addWidget(log_box_);
  layout->addWidget(log_group, 1);

  // ---------------- layout ----------------
  auto splitter = new QSplitter();
  splitter->addWidget(panel);
  splitter->addWidget(scene_);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({ 360, 1040 });
  setCentralWidget(splitter);

  connect(scene_, &SceneView::edgeSelected, this, &MainWindow::onEdgeSelectedInScene);
}

// ---------------------------------------------------------------------------
// slots
// ---------------------------------------------------------------------------
void MainWindow::onImportClicked()
{
  QString path = QFileDialog::getOpenFileName(
      this, "选择工件 STEP 文件", sample_step_path_.isEmpty() ? QDir::homePath() :
      QFileInfo(sample_step_path_).absolutePath(),
      "模型 (*.step *.stp *.brep);;所有文件 (*)");
  if (path.isEmpty())
    return;
  import_btn_->setEnabled(false);
  pending_model_path_ = path;
  pending_pose_ = Eigen::Isometry3d::Identity();
  pending_pose_.translation() =
      Eigen::Vector3d(place_x_->value(), place_y_->value(), place_z_->value());
  pending_pose_.linear() = Eigen::AngleAxisd(place_yaw_->value() * M_PI / 180.0,
                                             Eigen::Vector3d::UnitZ()).toRotationMatrix();
  log(QString("导入 %1 ...").arg(path));
  bridge_->sendImportModel(path, place_x_->value(), place_y_->value(), place_z_->value(),
                           place_yaw_->value() * M_PI / 180.0);
}

void MainWindow::onImportFinished(bool ok, QString model_id, QString message, int edge_count)
{
  import_btn_->setEnabled(true);
  log(QString("导入%1: %2").arg(ok ? "成功" : "失败").arg(message));
  if (!ok)
    return;

  // local STEP load for display: solid mesh (display tessellation)
  const auto edges = bridge_->lastImportedEdges();
  std::vector<rosim::core::EdgeData> edge_data;
  edge_data.reserve(edges.size());
  for (const auto& e : edges)
  {
    rosim::core::EdgeData d;
    d.index = e.edge_index;
    d.length = e.length;
    d.closed = e.closed;
    d.polyline.reserve(e.polyline.size());
    for (const auto& p : e.polyline)
      d.polyline.emplace_back(p.x, p.y, p.z);
    edge_data.push_back(std::move(d));
  }

  rosim::core::MeshData solid;
  try
  {
    const auto shape = rosim::core::OccGeometryService::loadShape(
        pending_model_path_.toStdString());
    rosim::core::OccGeometryService::tessellate(shape, 3e-4, 0.35, solid);
    model_paths_[model_id] = pending_model_path_;
  }
  catch (const std::exception& ex)
  {
    log(QString("本地 STEP 显示加载失败: %1").arg(ex.what()));
  }

  scene_->clearWorkpieces();
  scene_->addWorkpiece(model_id.toStdString(), solid, edge_data, pending_pose_);
  current_model_ = model_id;

  // rebuild tree
  tree_->clear();
  auto model_item = new QTreeWidgetItem(tree_);
  model_item->setText(0, model_id);
  model_item->setFlags(model_item->flags() & ~Qt::ItemIsSelectable);
  for (const auto& e : edge_data)
  {
    auto item = new QTreeWidgetItem(model_item);
    item->setText(0, QString("边 %1%2").arg(e.index).arg(e.closed ? " ○" : ""));
    item->setText(1, QString("%1 mm").arg(e.length * 1000.0, 0, 'f', 1));
    item->setData(0, Qt::UserRole, e.index);
  }
  tree_->expandAll();
  log(QString("工件 %1 已显示，共 %2 条边 — 鼠标悬停高亮、点击选择").arg(model_id).arg(edge_count));
}

void MainWindow::onEdgeSelectedInScene(int index)
{
  if (index < 0)
    return;
  selected_edge_ = index;
  scene_->setHighlightedEdge(index);
  // sync tree selection
  for (QTreeWidgetItemIterator it(tree_); *it; ++it)
  {
    if ((*it)->data(0, Qt::UserRole).toInt() == index)
    {
      tree_->blockSignals(true);
      tree_->setCurrentItem(*it);
      tree_->blockSignals(false);
      break;
    }
  }
  follow_btn_->setEnabled(!current_model_.isEmpty());
  log(QString("选择边 %1").arg(index));
}

void MainWindow::onTreeSelectionChanged()
{
  const auto items = tree_->selectedItems();
  if (items.isEmpty())
    return;
  const int index = items.first()->data(0, Qt::UserRole).toInt();
  if (index < 0)
    return;
  selected_edge_ = index;
  scene_->setHighlightedEdge(index);
  follow_btn_->setEnabled(!current_model_.isEmpty());
}

void MainWindow::onFollowClicked()
{
  if (current_model_.isEmpty() || selected_edge_ < 0)
    return;
  setBusy(true);
  log(QString("随边规划: model=%1 edge=%2 ...")
          .arg(current_model_)
          .arg(selected_edge_));
  bridge_->sendFollowEdge(current_model_, static_cast<uint32_t>(selected_edge_),
                          static_cast<uint8_t>(orient_combo_->currentData().toInt()),
                          tool_offset_spin_->value(), approach_spin_->value(),
                          spacing_spin_->value(), tilt_spin_->value() * M_PI / 180.0);
}

void MainWindow::onStopClicked()
{
  log("请求取消 ...");
  bridge_->cancelFollowEdge();
}

void MainWindow::onSpeedChanged(int value)
{
  speed_label_->setText(QString("%1 %").arg(value));
  bridge_->sendSpeedScale(value / 100.0);
}

void MainWindow::onFollowFeedback(float progress, QString phase)
{
  progress_->setValue(static_cast<int>(progress * 100));
  phase_label_->setText(phase);
}

void MainWindow::onFollowResult(bool ok, QString message, double cartesian_fraction)
{
  setBusy(false);
  progress_->setValue(ok ? 100 : progress_->value());
  phase_label_->setText(ok ? "done" : "failed");
  log(QString("随边%1: %2 (cartesian fraction=%3)")
          .arg(ok ? "完成" : "失败")
          .arg(message)
          .arg(cartesian_fraction, 0, 'f', 2));
}

void MainWindow::onJointState(sensor_msgs::msg::JointState state)
{
  // live angle readouts beside the jog buttons (before the msg is moved away)
  for (size_t i = 0; i < state.name.size() && i < state.position.size(); ++i)
  {
    auto it = jog_value_labels_.find(QString::fromStdString(state.name[i]));
    if (it != jog_value_labels_.end())
      it->second->setText(QString::asprintf("%.1f°", state.position[i] * 180.0 / M_PI));
  }
  robot_->onJointState(std::move(state));
  render_dirty_ = true;
}

void MainWindow::onHomeClicked()
{
  if (manual_pending_)
    return;
  manual_pending_ = true;
  home_btn_->setEnabled(false);
  log("回 Home：OMPL 规划避障路径 ...");
  bridge_->sendGoHome();
}

void MainWindow::onGoHomeResult(bool ok, QString message)
{
  manual_pending_ = false;
  home_btn_->setEnabled(true);
  log(QString("回Home%1: %2").arg(ok ? "启动" : "失败").arg(message));
}

void MainWindow::onJogResult(bool ok, QString message)
{
  manual_pending_ = false;
  if (!ok)
    log(QString("点动失败: %1").arg(message));
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
void MainWindow::log(const QString& line)
{
  log_box_->appendPlainText(
      QString("[%1] %2").arg(QTime::currentTime().toString("HH:mm:ss")).arg(line));
}

void MainWindow::setBusy(bool busy)
{
  follow_btn_->setEnabled(!busy && !current_model_.isEmpty() && selected_edge_ >= 0);
  stop_btn_->setEnabled(busy);
  import_btn_->setEnabled(!busy);
}

}  // namespace rosim::viewer
