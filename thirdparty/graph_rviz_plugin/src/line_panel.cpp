#include <graph_rviz_plugin/line_panel.hpp>

namespace graph_rviz_plugin
{

LinePanel::LinePanel(QWidget *parent) :
    rviz::Panel(parent),
    nh_(std::make_shared<ros::NodeHandle>()),
    start_pause_button_(new QPushButton),
    topic_button_(new QPushButton("Topics")),
    stop_button_(new QPushButton("Stop")),
    graph_settings_button_(new QPushButton("Graph settings")),
    export_button_(new QPushButton("Export")),
    graph_refresh_timer_(new QTimer(this)),
    plot_(new QCustomPlot)
{
  connect(this, &LinePanel::enable, this, &LinePanel::setEnabled);
  setName("Line graph");
  setObjectName(getName());

  yaxis_rescale_auto_ = true;
  window_time_enable_ = false;
  legend_enable_ = true;
  graph_stopped_ = true;
  y_max_ = 10;
  refresh_freq_ = 40;

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &LinePanel::displayMessageBox, this, &LinePanel::displayMessageBoxHandler);

  connect(start_pause_button_, &QPushButton::clicked, this, &LinePanel::startPauseClicked);
  connect(stop_button_, &QPushButton::clicked, this, &LinePanel::stopClicked);
  connect(topic_button_, &QPushButton::clicked, this, &LinePanel::topicsSelectionClicked);
  connect(graph_settings_button_, &QPushButton::clicked, this, &LinePanel::graphSettingsClicked);
  QPushButton *settings_button = new QPushButton("Settings");
  connect(settings_button, &QPushButton::clicked, this, &LinePanel::settingsClicked);
  connect(export_button_, &QPushButton::clicked, this, &LinePanel::exportClicked);
  QPushButton *reset_button = new QPushButton("Reset");
  connect(reset_button, &QPushButton::clicked, this, &LinePanel::resetClicked);

  start_pause_button_->setText("Start");

  QHBoxLayout *button_layout = new QHBoxLayout();
  button_layout->addWidget(start_pause_button_);
  button_layout->addWidget(stop_button_);
  button_layout->addStretch(1);
  button_layout->addWidget(topic_button_);
  button_layout->addWidget(graph_settings_button_);
  button_layout->addWidget(settings_button);
  button_layout->addWidget(export_button_);
  button_layout->addStretch(1);
  button_layout->addWidget(reset_button);

  QVBoxLayout *layout = new QVBoxLayout();
  setLayout(layout);
  layout->addLayout(button_layout);
  layout->addWidget(plot_);

  connect(graph_refresh_timer_, &QTimer::timeout, this, &LinePanel::graphUpdate);

  QFont legendFont = font();
  legendFont.setPointSize(9);
  plot_->legend->setFont(legendFont);
  plot_->legend->setBrush(QBrush(Qt::GlobalColor::white));

  start_pause_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  graph_settings_button_->setEnabled(false);
  export_button_->setEnabled(false);

  start_pause_button_->setMinimumWidth(45);
  stop_button_->setMinimumWidth(45);
  topic_button_->setMinimumWidth(50);
  graph_settings_button_->setMinimumWidth(95);
  settings_button->setMinimumWidth(70);
  export_button_->setMinimumWidth(50);
  reset_button->setMinimumWidth(45);
}

LinePanel::~LinePanel()
{
  nh_->shutdown();
  graph_refresh_timer_->stop();
}

void LinePanel::graphInit()
{
  int graph_index = 0;
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    if(displayed_topics_.at(i)->topic_data_.size() < 2){
      plot_->addGraph();
      plot_->graph(graph_index)->removeFromLegend();
      plot_->graph(graph_index)->setName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
      plot_->graph(graph_index)->addToLegend();
      graph_index++;
    }
    else{
      for(unsigned j = 0; j < displayed_topics_.at(i)->topic_data_.size(); j++){
        if(plot_->graphCount() < graph_index + 1){
            plot_->addGraph();
        }
        plot_->graph(graph_index)->removeFromLegend();
        plot_->graph(graph_index)->setName(QString::fromStdString(displayed_topics_.at(i)->topic_name_ + "/" + std::to_string(j)));
        plot_->graph(graph_index)->addToLegend();
        graph_index++;
      }
    }
  }
}

void LinePanel::graphSettingsUpdate()
{
  int graph_index = 0;
  QColor qColor;
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    int topic_data_size = displayed_topics_.at(i)->topic_data_.size();
    if(topic_data_size < 2){
      //transparent == HSV colormap, HSV is not used in single msg
      if(displayed_topics_.at(i)->color_ == Qt::GlobalColor::transparent){
        qColor = Qt::GlobalColor::black;
      }
      else {
        qColor = displayed_topics_.at(i)->color_;
      }
      plot_->graph(graph_index)->setPen(QPen(qColor, displayed_topics_.at(i)->thickness_,
                                             displayed_topics_.at(i)->style_));
      plot_->graph(graph_index)->setLineStyle(displayed_topics_.at(i)->line_style_);
      plot_->graph(graph_index)->setVisible(displayed_topics_.at(i)->displayed_);
      graph_index++;
    }
    else {
      for (int j = 0; j < topic_data_size; j++) {
        if(displayed_topics_.at(i)->color_ == Qt::GlobalColor::transparent){ //transparent == HSV colormap
          qColor.setHsvF(1.0 / topic_data_size * j, 1.0, 1.0, 1.0);
        }
        else{
          qColor = displayed_topics_.at(i)->color_;
        }
        plot_->graph(graph_index)->setPen(QPen(qColor, displayed_topics_.at(i)->thickness_,
                                               displayed_topics_.at(i)->style_));
        plot_->graph(graph_index)->setLineStyle(displayed_topics_.at(i)->line_style_);
        plot_->graph(graph_index)->setVisible(displayed_topics_.at(i)->displayed_);
        graph_index++;
      }
    }
  }

  plot_->replot();
}

void LinePanel::graphUpdate()
{
  int graph_index = 0;
  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    if (displayed_topics_.at(i)->data_update_ == true)
    {
      QVector<QVector<double>> topic_data = displayed_topics_.at(i)->getTopicData();
      QVector<double> topic_time = displayed_topics_.at(i)->getTopicTime();

      if (yaxis_rescale_auto_ == true)
        plot_->yAxis->rescale(true);
      else
        plot_->yAxis->setRange(y_min_, y_max_);

      if (window_time_enable_ == false)
        plot_->xAxis->rescale(true);
      else
      {
        if (!topic_time.empty())
          plot_->xAxis->setRange(topic_time.last(), w_time_, Qt::AlignRight);
        else
          plot_->xAxis->setRange(5, w_time_, Qt::AlignRight);
      }

      if(plot_->graphCount() < graph_index + topic_data.size()){
          graphInit();
          graphSettingsUpdate();
      }
      if(topic_data.size() == 1){
        plot_->graph(graph_index)->setData(topic_time, topic_data[0], true);
        graph_index++;
      }
      else {
        for(auto & data : topic_data){
          plot_->graph(graph_index)->setData(topic_time, data, true);
          graph_index++;
        }
      }

      displayed_topics_.at(i)->data_update_ = false;
      plot_->replot();
    }
  }
}

void LinePanel::displayMessageBoxHandler(const QString title,
                                         const QString message,
                                         const QString info_msg,
                                         const QMessageBox::Icon icon)
{
  const bool old(isEnabled());
  Q_EMIT setEnabled(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(message);
  msg_box.setInformativeText(info_msg);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  Q_EMIT setEnabled(old);
}

void LinePanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);

  unsigned i(0);
  {
    bool tmp;

    if (config.mapGetBool("window_time_enable", &tmp))
      window_time_enable_ = tmp;

    if (config.mapGetBool("yaxis_rescale_auto", &tmp))
      yaxis_rescale_auto_ = tmp;

    if (config.mapGetBool("legend_enable", &tmp))
      legend_enable_ = tmp;
  }

  {
    float tmp;

    if (config.mapGetFloat("w_time", &tmp))
      w_time_ = (double) tmp;

    if (config.mapGetFloat("y_min", &tmp))
      y_min_ = (double) tmp;

    if (config.mapGetFloat("y_max", &tmp))
      y_max_ = (double) tmp;
  }

  {
    int tmp;

    if (config.mapGetInt("refresh_freq", &tmp))
      refresh_freq_ = std::abs(tmp);
  }

  {
    QString tmp;

    if (config.mapGetString("export_directory", &tmp))
      export_directory_ = tmp;
  }

  while (1)
  {
    QString topic_name;
    QString topic_type;
    int topic_thickness;
    int color_index;
    int style_index;

    if (!config.mapGetString("topic_" + QString::number(i) + "_name", &topic_name))
      break;

    if (!config.mapGetString("topic_" + QString::number(i) + "_type", &topic_type))
      break;

    if (!config.mapGetInt("topic_" + QString::number(i) + "_thickness", &topic_thickness))
      break;

    if (!config.mapGetInt("topic_" + QString::number(i) + "_color", &color_index))
      break;

    if (!config.mapGetInt("topic_" + QString::number(i) + "_style", &style_index))
      break;

    std::shared_ptr<TopicData> topic_data = std::make_shared<TopicData>(topic_name.toStdString(),
                                                                        topic_type.toStdString(), nh_);
    topic_data->thickness_ = topic_thickness;
    topic_data->color_ = topic_color_class_.getColorFromIndex(color_index);
    topic_data->style_ = topic_style_class_.getStyleFromIndex(style_index);
    displayed_topics_.push_back(topic_data);
    ++i;
  }

  if (displayed_topics_.empty())
    return;

  start_pause_button_->setEnabled(true);
  stop_button_->setEnabled(true);
  graph_settings_button_->setEnabled(true);
  graphInit();
  graphSettingsUpdate();
  Q_EMIT enableLegend(legend_enable_);

  startPauseClicked(); //TODO: add parameter for auto start
}

void LinePanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("window_time_enable", window_time_enable_.load());
  config.mapSetValue("yaxis_rescale_auto", yaxis_rescale_auto_.load());
  config.mapSetValue("legend_enable", legend_enable_.load());
  config.mapSetValue("w_time", w_time_);
  config.mapSetValue("y_min", y_min_);
  config.mapSetValue("y_max", y_max_);
  config.mapSetValue("refresh_freq", refresh_freq_);
  config.mapSetValue("export_directory", export_directory_);

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    config.mapSetValue("topic_" + QString::number(i) + "_name",
                       QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    config.mapSetValue("topic_" + QString::number(i) + "_type",
                       QString::fromStdString(displayed_topics_.at(i)->topic_type_));
    config.mapSetValue("topic_" + QString::number(i) + "_thickness", displayed_topics_.at(i)->thickness_);
    config.mapSetValue("topic_" + QString::number(i) + "_color",
                       topic_color_class_.getIndexFromColor(displayed_topics_.at(i)->color_));
    config.mapSetValue("topic_" + QString::number(i) + "_style",
                       topic_style_class_.getIndexFromStyle(displayed_topics_.at(i)->style_));
  }
}

void LinePanel::startPauseClicked()
{
  if ((graph_refresh_timer_->isActive()) == false)
  {
    graph_refresh_timer_->start((1.0 / refresh_freq_) * 1000.0); // Hz > seconds > milliseconds
    start_pause_button_->setText("Pause");
    plot_->setInteraction(QCP::iRangeZoom, false);
    plot_->setInteraction(QCP::iRangeDrag, false);
    topic_button_->setEnabled(false);
    stop_button_->setEnabled(true);
    export_button_->setEnabled(false);

    if (graph_stopped_ == true)
    {
      graph_stopped_ = false;
      plot_->clearGraphs();
      plot_->clearPlottables();
      plot_->replot();
      graphInit();

      for (unsigned i = 0; i < displayed_topics_.size(); i++)
      {
        displayed_topics_.at(i)->clearData();
        displayed_topics_.at(i)->startRefreshData();
      }

      graphSettingsUpdate();
      graphUpdate();
    }

    return;
  }

  else
  {
    graph_refresh_timer_->stop();
    start_pause_button_->setText("Start");
    plot_->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
    export_button_->setEnabled(true);
    return;
  }
}

void LinePanel::stopClicked()
{
  topic_button_->setEnabled(true);

  if ((graph_refresh_timer_->isActive()) == true)
    startPauseClicked();

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
    displayed_topics_.at(i)->stopRefreshData();

  graph_stopped_ = true;
}

void LinePanel::resetClicked()
{
  if ((graph_refresh_timer_->isActive()) == true)
    startPauseClicked();

  for (unsigned i = 0; i < displayed_topics_.size(); i++)
  {
    displayed_topics_.at(i)->stopRefreshData();
    displayed_topics_.at(i)->clearData();
  }

  displayed_topics_.clear();
  plot_->legend->setVisible(false);
  plot_->clearGraphs();
  plot_->clearPlottables();
  plot_->replot();

  start_pause_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  graph_settings_button_->setEnabled(false);
  topic_button_->setEnabled(true);

  Q_EMIT configChanged();
}

void LinePanel::exportClicked()
{
  QString file_name = QFileDialog::getSaveFileName(this, tr("Save file"), export_directory_ + "/graph.png",
                                                   tr("PNG (*.png);; JPG (*.jpg) ;; PDF (*.pdf)"));

  if (file_name.isEmpty())
    return;

  QFileInfo path = QFile(file_name);

  if (path.completeSuffix() == "png")
    plot_->savePng(file_name);
  else if (path.completeSuffix() == "jpg")
    plot_->saveJpg(file_name);
  else
    plot_->savePdf(file_name);

  export_directory_ = path.absolutePath();

  Q_EMIT configChanged();
}

void LinePanel::topicsSelectionClicked()
{
  std::vector<std::string> allowed_topics;
  allowed_topics.emplace_back("std_msgs/Bool");
  allowed_topics.emplace_back("std_msgs/Int8");
  allowed_topics.emplace_back("std_msgs/Int8MultiArray");
  allowed_topics.emplace_back("std_msgs/UInt8");
  allowed_topics.emplace_back("std_msgs/UInt8MultiArray");
  allowed_topics.emplace_back("std_msgs/Int16");
  allowed_topics.emplace_back("std_msgs/Int16MultiArray");
  allowed_topics.emplace_back("std_msgs/UInt16");
  allowed_topics.emplace_back("std_msgs/UInt16MultiArray");
  allowed_topics.emplace_back("std_msgs/Int32");
  allowed_topics.emplace_back("std_msgs/Int32MultiArray");
  allowed_topics.emplace_back("std_msgs/UInt32");
  allowed_topics.emplace_back("std_msgs/UInt32MultiArray");
  allowed_topics.emplace_back("std_msgs/Int64");
  allowed_topics.emplace_back("std_msgs/Int64MultiArray");
  allowed_topics.emplace_back("std_msgs/UInt64");
  allowed_topics.emplace_back("std_msgs/UInt64MultiArray");
  allowed_topics.emplace_back("std_msgs/Float32");
  allowed_topics.emplace_back("std_msgs/Float32MultiArray");
  allowed_topics.emplace_back("std_msgs/Float64");
  allowed_topics.emplace_back("std_msgs/Float64MultiArray");

  SelectionTopics *topic_window = new SelectionTopics(nh_, displayed_topics_, allowed_topics, false);

  if (topic_window->supported_topics_.empty())
  {
    Q_EMIT displayMessageBox("No supported topic", "Error with topics, no supported topics found.", "",
                             QMessageBox::Icon::Warning);
    return;
  }

  if (topic_window->exec())
  {
    if ((graph_refresh_timer_->isActive()) == true)
      startPauseClicked();

    resetClicked();
    displayed_topics_ = topic_window->displayed_topics_;

    for (unsigned i = 0; i < displayed_topics_.size(); i++) {
      displayed_topics_.at(i)->color_ = topic_color_class_.getColorFromIndex(
            i % ((topic_color_class_.colors_list_).size()));
      displayed_topics_.at(i)->style_ = topic_style_class_.getStyleFromIndex(
              i % ((topic_style_class_.style_list_).size()));
    }

    graphInit();
    graphSettingsUpdate();
    Q_EMIT configChanged();
    Q_EMIT enableLegend(legend_enable_);

    if (displayed_topics_.empty() == false)
    {
      start_pause_button_->setEnabled(true);
      graph_settings_button_->setEnabled(true);
    }
  }
}

void LinePanel::graphSettingsClicked()
{
  GraphSettings *configure_topics = new GraphSettings(displayed_topics_);

  if (configure_topics->exec())
    graphSettingsUpdate();
}

void LinePanel::settingsClicked()
{
  Settings *configure_graph = new Settings(yaxis_rescale_auto_, window_time_enable_,
                                           legend_enable_, y_min_, y_max_, w_time_, refresh_freq_);

  if (!configure_graph->exec())
    return;

  window_time_enable_ = configure_graph->window_time_enable_;
  yaxis_rescale_auto_ = configure_graph->scale_auto_;
  w_time_ = configure_graph->w_time_;
  y_min_ = configure_graph->y_min_;
  y_max_ = configure_graph->y_max_;
  legend_enable_ = configure_graph->legend_enable_;
  Q_EMIT enableLegend(legend_enable_);
  refresh_freq_ = configure_graph->refresh_freq_; // Hz
  Q_EMIT configChanged();

  // Restart the timer in case we are recording
  if (graph_refresh_timer_->isActive())
  {
    graph_refresh_timer_->stop();
    graph_refresh_timer_->start((1.0 / configure_graph->refresh_freq_) * 1000.0); // Hz > seconds > milliseconds
  }
}

void LinePanel::enableLegend(bool legend_enable)
{
  plot_->legend->setVisible(legend_enable);
}

}
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(graph_rviz_plugin::LinePanel, rviz::Panel)

