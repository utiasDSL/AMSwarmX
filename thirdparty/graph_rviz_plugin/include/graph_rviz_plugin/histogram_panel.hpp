#ifndef GRAPH_RVIZ_PLUGIN_HISTOGRAM_PANEL_HPP
#define GRAPH_RVIZ_PLUGIN_HISTOGRAM_PANEL_HPP

#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <graph_rviz_plugin/qcustomplot.h>
#include <graph_rviz_plugin/selection_topics.hpp>
#include <mutex>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#include <sensor_msgs/Image.h>

#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace graph_rviz_plugin
{

class HistogramPanel : public rviz::Panel
{
Q_OBJECT

public:
  HistogramPanel(QWidget *parent = nullptr);
  virtual ~HistogramPanel();

Q_SIGNALS:
  void subscribeToTopic(const QString);
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void updateChartSlot();
  void topicSelectionSlot();
  void subscribeToTopicSlot(const QString topic);

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Subscriber sub_;

  std::atomic<bool> updating_;
  std::mutex data_ticks_mutex_;

  int16_t bins_value_;
  bool grayscale_ = true; // true <=> grayscale; false <=> color images
  double histogram_max_counter_;

  QString topic_;
  QVector<double> ticks_;
  QVector<double> data_; // Grayscale channel histogram data
  QVector<double> blue_channel_data_; // Blue channel histogram data
  QVector<double> green_channel_data_; // Green channel histogram data
  QVector<double> red_channel_data_; // Red channel histogram data

  QPushButton *start_stop_;
  QComboBox *graph_refresh_frequency_;
  QComboBox *bins_selection_;
  QTimer *graph_refresh_timer_;

  QCustomPlot *custom_plot_;
  QCPBars *bars_;
  QCPBars *bars_red_;
  QCPBars *bars_blue_;
  QCPBars *bars_green_;

};

}

#endif
