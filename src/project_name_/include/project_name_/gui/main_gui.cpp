#include <project_name_/gui/main_gui.h>

#include <project_name_/main_ctrl.h>

MainWindow::MainWindow(MainController *main_ctrl, QWidget *parent) : QMainWindow(parent)
{
  this->main_ctrl = main_ctrl;

  std::vector<std::string> priority_name = {"IdlePriority", "LowestPriority", "LowPriority", "NormalPriority", "HighPriority", "HighestPriority", "TimeCriticalPriority", "InheritPriority"};
  QThread::Priority priority = QThread::currentThread()->priority();

  //this->resize(400,350);
  this->setWindowTitle("Main window");

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  this->font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  this->font2 = QFont("Ubuntu", 13, QFont::DemiBold);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  // --------------------------------------------

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addWidget(createModeFrame());
  main_layout->addWidget(createUtilsFrame());

  createActions();
  createMenu();

  QObject::connect( this, SIGNAL(modeChangedSignal(const QString &)), this, SLOT(modeChangedSlot(const QString &)) );
  emit modeChangedSignal("IDLE");
}



MainWindow::~MainWindow()
{}

void MainWindow::modeChangedSlot(const QString &mode_name)
{
  robot_mode_le->setText(mode_name);

  if (mode_name.compare("idle", Qt::CaseInsensitive)==0) robot_mode_le->setStyleSheet("color:rgb(255,160,52);");
  else if (mode_name.compare("freedrive", Qt::CaseInsensitive)==0) robot_mode_le->setStyleSheet("color:rgb(0,200,0);");
  else if (mode_name.compare("CART_VEL_CTRL", Qt::CaseInsensitive)==0) robot_mode_le->setStyleSheet("color:rgb(255,65,195);");
  else if (mode_name.compare("GOTO START", Qt::CaseInsensitive)==0) robot_mode_le->setStyleSheet("color:rgb(170,85,0);");
}

QFrame *MainWindow::createUtilsFrame()
{
  goto_start_pose_btn = new QPushButton("goto start pose");
  goto_start_pose_btn->setFont(font1);
  QObject::connect( goto_start_pose_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->gotoStartPose(); } );

  bias_ft_sensors_btn = new QPushButton("bias F/T sensors");
  bias_ft_sensors_btn->setFont(font1);
  QObject::connect( bias_ft_sensors_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->biasFTSensors(); } );

  log_data_ckbox = new QCheckBox("enable data logging");
  log_data_ckbox->setFont(font1);
  QObject::connect( log_data_ckbox, &QCheckBox::stateChanged, this, [this]()
  { this->main_ctrl->setLogging(log_data_ckbox->isChecked()); });

  save_log_data = new QPushButton("save logged data");
  save_log_data->setFont(font1);
  QObject::connect( save_log_data, &QPushButton::clicked, this, [this]()
  {
    std::string filename = QFileDialog::getSaveFileName(this, tr("Save logged data"), this->main_ctrl->default_data_path.c_str(), "Binary files (*.bin)").toStdString();
    if (filename.empty()) return;
    this->main_ctrl->saveLoggedData(filename);
  });

  QVBoxLayout *utils_layout = new QVBoxLayout;
  utils_layout->addWidget(goto_start_pose_btn);
  utils_layout->addWidget(bias_ft_sensors_btn);
  utils_layout->addWidget(log_data_ckbox);
  utils_layout->addWidget(save_log_data);
  utils_layout->addStretch(0);

  QFrame *utils_frame = new QFrame;
  utils_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  utils_frame->setLineWidth(2);
  utils_frame->setLayout(utils_layout);

  return utils_frame;
}

QFrame *MainWindow::createModeFrame()
{
  QVBoxLayout *robot_mode_layout = new QVBoxLayout;

  QLabel *robot_mode_lb = new QLabel("Robot mode");
  robot_mode_lb->setAlignment(Qt::AlignCenter);
  robot_mode_lb->setFont(font1);
  robot_mode_lb->setStyleSheet("color:rgb(0,0,255); background-color:rgba(210, 210, 210, 100);");
  robot_mode_le = new QLineEdit("");
  robot_mode_le->setAlignment(Qt::AlignCenter);
  robot_mode_le->setFont(font1);
  robot_mode_le->setMinimumWidth(200);
  robot_mode_le->setReadOnly(true);

  QHBoxLayout *rm1_layout = new QHBoxLayout;
    rm1_layout->addWidget(robot_mode_lb);
    rm1_layout->addWidget(robot_mode_le);
    rm1_layout->addStretch(0);
  robot_mode_layout->addLayout(rm1_layout);

  freedrive_btn = new QPushButton("Freedrive");
  freedrive_btn->setFont(font1);
  QObject::connect( freedrive_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->setFreedriveMode(); } );

  idle_btn = new QPushButton("Idle");
  idle_btn->setFont(font1);
  QObject::connect( idle_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->setIdleMode(); } );

  run_sim_btn = new QPushButton("Run Sim");
  run_sim_btn->setFont(font1);
  QObject::connect( run_sim_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->runSimulation(); } );

  QVBoxLayout *modes_btn_layout = new QVBoxLayout;
  modes_btn_layout->addWidget(freedrive_btn);
  modes_btn_layout->addWidget(idle_btn);
  modes_btn_layout->addWidget(run_sim_btn);
  QHBoxLayout *modes_btn_layout2 = new QHBoxLayout;
  modes_btn_layout2->addLayout(modes_btn_layout);
  modes_btn_layout2->addStretch();

  robot_mode_layout->addLayout(modes_btn_layout2);

  QFrame *robot_mode_frame = new QFrame;
  robot_mode_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  robot_mode_frame->setLineWidth(2);
  robot_mode_frame->setLayout(robot_mode_layout);

  return robot_mode_frame;
}

QFrame *MainWindow::createDampFrame()
{
  // QLabel *Dmin_lb, *Dmax_lb;
  // QLineEdit *Dmin_le;
  //
  // QComboBox *damp_adapt_method_cmbox;
  // damp_adapt_method_cmbox->setFont(font3);
  // damp_adapt_method_cmbox->addItem("power");
  // damp_adapt_method_cmbox->addItem("vel");
  // damp_adapt_method_cmbox->addItem("force");
  // damp_adapt_method_cmbox->addItem("min");
  // damp_adapt_method_cmbox->addItem("max");
  // damp_adapt_method_cmbox->addItem("mean");
  // QObject::connect( damp_adapt_method_cmbox, &QComboBox::currentTextChanged, this, [this](const QString &method)
  // {
  //
  // });
  // damp_adapt_method_cmbox->setCurrentText("power");
  // emit damp_adapt_method_cmbox->currentTextChanged("power");
}

void MainWindow::createActions()
{
  gui_::ViewWrenchDialog *view_lh_wrench_dialog = new gui_::ViewWrenchDialog([this](){ return main_ctrl->getLeftHandleWrench(); }, 0, this);
  view_lh_wrench_dialog->setTitle("Left Handle wrench");
  view_lh_wrench_act = new QAction(tr("View left handle wrench"), this);
  view_lh_wrench_act->setStatusTip(tr("Opens a window displaying the wrench measured from the left handle."));
  QObject::connect( view_lh_wrench_act, &QAction::triggered, this, [view_lh_wrench_dialog](){ view_lh_wrench_dialog->launch(); } );

  // QAction *view_rh_wrench_act;
  gui_::ViewWrenchDialog *view_rh_wrench_dialog = new gui_::ViewWrenchDialog([this](){ return main_ctrl->getRightHandleWrench(); }, 0, this);
  view_rh_wrench_dialog->setTitle("Right Handle wrench");
  view_rh_wrench_act = new QAction(tr("View right handle wrench"), this);
  view_rh_wrench_act->setStatusTip(tr("Opens a window displaying the wrench measured from the right handle."));
  QObject::connect( view_rh_wrench_act, &QAction::triggered, this, [view_rh_wrench_dialog](){ view_rh_wrench_dialog->launch(); } );
}

void MainWindow::createMenu()
{
  // =======   Create menus   ==========

  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
  // file_menu->addAction(train_win->load_train_data_act);
  // file_menu->addAction(load_model_act);
  // file_menu->addSeparator();

  QMenu *edit_menu = menu_bar->addMenu(tr("&Edit"));
  // edit_menu->addAction(set_start_pose_act);
  // edit_menu->addSeparator();

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
  // view_menu->addAction(view_joints_act);
  // view_menu->addAction(view_pose_act);
  view_menu->addAction(view_lh_wrench_act);
  view_menu->addAction(view_rh_wrench_act);
  // view_menu->addSeparator();
  // view_menu->addAction(train_win->plot_train_data_act);
  // view_menu->addAction(train_win->plot_demo_sim_data_act);

}
