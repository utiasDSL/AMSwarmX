#ifndef GRAPH_RVIZ_PLUGIN_SETTINGS_HPP
#define GRAPH_RVIZ_PLUGIN_SETTINGS_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>

namespace graph_rviz_plugin
{

class Settings : public QDialog
{
Q_OBJECT

public:
  Settings(bool scale_auto, bool window_time_enable, bool legend_enable, double y_min, double y_max, double w_time,
           double refresh_freq, QDialog *parent = 0);
  ~Settings();
  double y_min_;
  double y_max_;
  double w_time_;
  double refresh_freq_; // in Hz
  bool scale_auto_;
  bool window_time_enable_;
  bool legend_enable_;

protected Q_SLOTS:
  void yAxisAutoscale(bool checked);
  void xAxisWindowTime(bool checked);
  void yMinChanged(double y_min);
  void yMaxChanged(double y_max);
  void okClicked();

private:
  QDoubleSpinBox *y_min_double_spin_box_;
  QDoubleSpinBox *y_max_double_spin_box_;
  QDoubleSpinBox *w_time_double_spin_box_;
  QComboBox *refresh_frequency_spin_box_;
  QCheckBox *legend_enable_button_;
};

}

#endif
