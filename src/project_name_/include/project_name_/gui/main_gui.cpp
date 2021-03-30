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
  // main_layout->addLayout(rh_layout);

  createMenu();

  QObject::connect( this, &MainWindow::modeChangedSignal, this, [this](const QString &mode_name){ this->robot_mode_le->setText(mode_name); } );
}

MainWindow::~MainWindow()
{}

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
  robot_mode_le->setMinimumWidth(300);
  robot_mode_le->setReadOnly(true);

  QHBoxLayout *rm1_layout = new QHBoxLayout;
    rm1_layout->addWidget(robot_mode_lb);
    rm1_layout->addWidget(robot_mode_le);
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

  QFrame *robot_mode_frame = new QFrame(central_widget);
  robot_mode_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  robot_mode_frame->setLineWidth(2);
  robot_mode_frame->setLayout(robot_mode_layout);

  return robot_mode_frame;
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
  // view_menu->addAction(view_wrench_act);
  // view_menu->addSeparator();
  // view_menu->addAction(train_win->plot_train_data_act);
  // view_menu->addAction(train_win->plot_demo_sim_data_act);

}
