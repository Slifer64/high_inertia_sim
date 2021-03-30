#ifndef GUI_MAINWINDOW_H
#define GUI_MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QThread>

#include <map>
#include <functional>

#include <project_name_/gui/wrench_ctrl_gui.h>

class MainController; // forward decleration

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit MainWindow(MainController *main_ctrl, QWidget *parent = 0);
  ~MainWindow();

signals:
  void closeSignal();
  void modeChangedSignal(const QString &mode_name);

public:

  MainController *main_ctrl;

  QFrame *createModeFrame();

  void createMenu();

  QLineEdit *robot_mode_le;
  QPushButton *freedrive_btn;
  QPushButton *idle_btn;
  QPushButton *run_sim_btn;

  QWidget *central_widget;
  QStatusBar *status_bar;

  QFont font1;
  QFont font2;

};

#endif // GUI_MAINWINDOW_H
