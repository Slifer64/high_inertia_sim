#include <project_name_/gui/wrench_ctrl_gui.h>

#include <QDebug>
#include <iostream>

WrenchCtrlGui::WrenchCtrlGui(QWidget *parent): QFrame(parent)
{
  double fmax = 5;
  double tmax = 1;

  fx_slider = new TrainSlider(fmax, "fx");
  fy_slider = new TrainSlider(fmax, "fy");
  fz_slider = new TrainSlider(fmax, "fz");

  tx_slider = new TrainSlider(tmax, "tx");
  ty_slider = new TrainSlider(tmax, "ty");
  tz_slider = new TrainSlider(tmax, "tz");

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addWidget(fx_slider);
  main_layout->addWidget(fy_slider);
  main_layout->addWidget(fz_slider);
  main_layout->addWidget(tx_slider);
  main_layout->addWidget(ty_slider);
  main_layout->addWidget(tz_slider);

  QObject::connect(fx_slider, &TrainSlider::sliderValueChanged, this, [this](double value){ fx = value; });
  QObject::connect(fy_slider, &TrainSlider::sliderValueChanged, this, [this](double value){ fy = value; });
  QObject::connect(fz_slider, &TrainSlider::sliderValueChanged, this, [this](double value){ fz = value; });
  QObject::connect(tx_slider, &TrainSlider::sliderValueChanged, this, [this](double value){ tx = value; });
  QObject::connect(ty_slider, &TrainSlider::sliderValueChanged, this, [this](double value){ ty = value; });
  QObject::connect(tz_slider, &TrainSlider::sliderValueChanged, this, [this](double value){ tz = value; });

}

WrenchCtrlGui::~WrenchCtrlGui()
{

}
