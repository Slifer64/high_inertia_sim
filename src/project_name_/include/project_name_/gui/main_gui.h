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

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

signals:
  void closeSignal();

public:

  void createMenu();

  WrenchCtrlGui *lh_ctrl_;
  WrenchCtrlGui *rh_ctrl_;

  QWidget *central_widget;
  QStatusBar *status_bar;

};

#endif // GUI_MAINWINDOW_H
