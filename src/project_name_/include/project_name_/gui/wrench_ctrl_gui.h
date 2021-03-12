#ifndef WRENCH_CONTROL_GUI
#define WRENCH_CONTROL_GUI

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QSlider>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QMainWindow>
#include <QRadioButton>
#include <QDialogButtonBox>
#include <QButtonGroup>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

#include <project_name_/gui/train_slider.h>

class WrenchCtrlGui : public QFrame
{
  Q_OBJECT

public:
  WrenchCtrlGui(QWidget *parent=0);
  ~WrenchCtrlGui();

  arma::vec getWrench() const { return {fx, fy, fz, tx, ty, tz}; }

private:

  double fx;
  double fy;
  double fz;
  double tx;
  double ty;
  double tz;

  TrainSlider *fx_slider;
  TrainSlider *fy_slider;
  TrainSlider *fz_slider;
  TrainSlider *tx_slider;
  TrainSlider *ty_slider;
  TrainSlider *tz_slider;

private:



};

#endif // WRENCH_CONTROL_GUI
