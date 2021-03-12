#include <project_name_/gui/main_gui.h>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
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

  QFont font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  QFont font2 = QFont("Ubuntu", 13, QFont::DemiBold);

  QLabel *lh_label = new QLabel("Left handle ctrl");
  lh_label->setFont(font1);
  lh_label->setAlignment(Qt::AlignCenter);
  lh_ctrl_ = new WrenchCtrlGui;
  QVBoxLayout *lh_layout = new QVBoxLayout;
  lh_layout->addWidget(lh_label);
  lh_layout->addWidget(lh_ctrl_);

  QLabel *rh_label = new QLabel("Right handle ctrl");
  rh_label->setFont(font1);
  rh_label->setAlignment(Qt::AlignCenter);
  rh_ctrl_ = new WrenchCtrlGui;
  QVBoxLayout *rh_layout = new QVBoxLayout;
  rh_layout->addWidget(rh_label);
  rh_layout->addWidget(rh_ctrl_);

  // --------------------------------------------

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addLayout(lh_layout);
  main_layout->addLayout(rh_layout);

  createMenu();
}

MainWindow::~MainWindow()
{}

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
