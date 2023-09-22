#ifndef GRAPH_RVIZ_PLUGIN_SELECTION_TOPICS_HPP
#define GRAPH_RVIZ_PLUGIN_SELECTION_TOPICS_HPP

#include <deque>
#include <mutex>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <graph_rviz_plugin/topic_data.hpp>

namespace graph_rviz_plugin
{

class SelectionTopics : public QDialog
{
Q_OBJECT

public:
  SelectionTopics(std::shared_ptr<ros::NodeHandle> nh,
                  std::deque<std::shared_ptr<TopicData>> already_displayed_topics,
                  const std::vector<std::string> allowed_types,
                  const bool single_choice,
                  QDialog *parent = 0);
  ~SelectionTopics();
  std::deque<std::shared_ptr<TopicData>> displayed_topics_;
  ros::master::V_TopicInfo supported_topics_;

Q_SIGNALS:
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
  void okClicked();

private:
  void detectTopics();
  std::vector<QAbstractButton *> topic_buttons_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::deque<std::shared_ptr<TopicData>> already_displayed_topics_;
  const std::vector<std::string> allowed_types_;
  const bool single_choice_;
};

}

#endif
