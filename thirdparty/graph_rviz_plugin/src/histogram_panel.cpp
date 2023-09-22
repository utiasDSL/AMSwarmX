#include <graph_rviz_plugin/histogram_panel.hpp>

namespace graph_rviz_plugin
{

HistogramPanel::HistogramPanel(QWidget *parent) : rviz::Panel(parent),
                                                  nh_(std::make_shared<ros::NodeHandle>()),
                                                  updating_(false),
                                                  bins_value_(256)
{
  setName("Histogram");
  setObjectName(getName());
  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, &HistogramPanel::enable, this, &HistogramPanel::setEnabled);
  connect(this, &HistogramPanel::displayMessageBox, this, &HistogramPanel::displayMessageBoxHandler);
  connect(this, &HistogramPanel::subscribeToTopic, this, &HistogramPanel::subscribeToTopicSlot);

  QVBoxLayout *layout = new QVBoxLayout();
  setLayout(layout);

  graph_refresh_timer_ = new QTimer(this);
  graph_refresh_timer_->setInterval(100);  // milliseconds // 10 Hz
  connect(graph_refresh_timer_, &QTimer::timeout, this, &HistogramPanel::updateChartSlot);

  QHBoxLayout *buttons(new QHBoxLayout);
  start_stop_ = new QPushButton("Start");
  start_stop_->setToolTip("Start the histogram");
  start_stop_->setEnabled(false);
  connect(start_stop_, &QPushButton::clicked, this, [=]()
  {
    if (updating_)
    {
      graph_refresh_timer_->start();
      start_stop_->setText("Stop");
      start_stop_->setToolTip("Stop the histogram");
      custom_plot_->setInteraction(QCP::iRangeZoom, false);
      custom_plot_->setInteraction(QCP::iRangeDrag, false);
      updating_ = false;
    }
    else
    {
      graph_refresh_timer_->stop();
      start_stop_->setText("Start");
      start_stop_->setToolTip("Start the histogram");
      custom_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
      updating_ = true;
    }

  });

  graph_refresh_frequency_ = new QComboBox;
  QLabel *refresh_label(new QLabel("Refresh frequency:"));
  graph_refresh_frequency_->setToolTip("Change the histogram display frequency");
  refresh_label->setToolTip(graph_refresh_frequency_->toolTip());

  graph_refresh_frequency_->addItem("1 Hz");
  graph_refresh_frequency_->addItem("2 Hz");
  graph_refresh_frequency_->addItem("5 Hz");
  graph_refresh_frequency_->addItem("10 Hz");
  graph_refresh_frequency_->addItem("20 Hz");
  graph_refresh_frequency_->addItem("50 Hz");
  graph_refresh_frequency_->setCurrentIndex(3); // Display by default "5 Hz"

  connect(graph_refresh_frequency_, qOverload<int>(&QComboBox::currentIndexChanged), this, &Panel::configChanged);
  connect(graph_refresh_frequency_, qOverload<int>(&QComboBox::currentIndexChanged), this, [=](int index)
  {
    switch (index)
    {
      case 0:
      {
        graph_refresh_timer_->setInterval(1000 / 1);
        break;
      }
      case 1:
      {
        graph_refresh_timer_->setInterval(1000 / 2);
        break;
      }
      case 2:
      {
        graph_refresh_timer_->setInterval(1000 / 5);
        break;
      }
      case 3:
      {
        graph_refresh_timer_->setInterval(1000 / 10);
        break;
      }
      case 4:
      {
        graph_refresh_timer_->setInterval(1000 / 20);
        break;
      }
      case 5:
      {
        graph_refresh_timer_->setInterval(1000 / 50);
        break;
      }
      default:
      {
        graph_refresh_timer_->setInterval(1000 / 5); // Default to 5 Hz
        break;
      }
    }
  });

  custom_plot_ = new QCustomPlot;

  bars_ = new QCPBars(custom_plot_->xAxis, custom_plot_->yAxis);
  bars_->setAntialiased(false);
  bars_->setPen(QPen(QColor(0, 0, 0)));
  bars_->setBrush(QColor(128, 128, 128));
  bars_->setWidthType(QCPBars::WidthType::wtPlotCoords);
  bars_->setWidth(1);
  bars_->setVisible(true);

  bars_red_ = new QCPBars(custom_plot_->xAxis, custom_plot_->yAxis);
  bars_red_->setAntialiased(false);
  bars_red_->setPen(QPen(QColor(0, 0, 0)));
  bars_red_->setBrush(QColor(255, 0, 0));
  bars_red_->setWidthType(QCPBars::WidthType::wtPlotCoords);
  bars_red_->setWidth(1);
  bars_red_->setVisible(false);

  bars_green_ = new QCPBars(custom_plot_->xAxis, custom_plot_->yAxis);
  bars_green_->setAntialiased(false);
  bars_green_->setPen(QPen(QColor(0, 0, 0)));
  bars_green_->setBrush(QColor(0, 255, 0));
  bars_green_->setWidthType(QCPBars::WidthType::wtPlotCoords);
  bars_green_->setWidth(1);
  bars_green_->setVisible(false);

  bars_blue_ = new QCPBars(custom_plot_->xAxis, custom_plot_->yAxis);
  bars_blue_->setAntialiased(false);
  bars_blue_->setPen(QPen(QColor(0, 0, 0)));
  bars_blue_->setBrush(QColor(0, 0, 255));
  bars_blue_->setWidthType(QCPBars::WidthType::wtPlotCoords);
  bars_blue_->setWidth(1);
  bars_blue_->setVisible(false);

  for (int i(0); i < 255; ++i)
    ticks_.push_back(i);

  custom_plot_->yAxis->rescale();
  custom_plot_->xAxis->setRange(0, ticks_.back());
  custom_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

  bins_selection_ = new QComboBox;
  QLabel *bins_selection_label = new QLabel("Bins selection:");
  bins_selection_->setToolTip("Change the bins number used to compute the histogram");
  bins_selection_label->setToolTip(bins_selection_->toolTip());

  bins_selection_->addItem("16");
  bins_selection_->addItem("32");
  bins_selection_->addItem("64");
  bins_selection_->addItem("128");
  bins_selection_->addItem("256");
  bins_selection_->addItem("512");
  bins_selection_->addItem("1024");
  bins_selection_->addItem("2048");
  bins_selection_->addItem("4096");
  bins_selection_->addItem("8192");
  bins_selection_->addItem("16384");

  connect(bins_selection_, qOverload<int>(&QComboBox::currentIndexChanged), this, &Panel::configChanged);
  connect(bins_selection_, qOverload<int>(&QComboBox::currentIndexChanged), this, [=](const int index)
  {
    switch (index)
    {
      case 0:
      {
        bins_value_ = 16;
        break;
      }
      case 1:
      {
        bins_value_ = 32;

        break;
      }
      case 2:
      {
        bins_value_ = 64;
        break;
      }
      case 3:
      {
        bins_value_ = 128;
        break;
      }
      case 4:
      {
        bins_value_ = 256;
        break;
      }
      case 5:
      {
        bins_value_ = 512;
        break;
      }
      case 6:
      {
        bins_value_ = 1024;
        break;
      }
      case 7:
      {
        bins_value_ = 2048;
        break;
      }
      case 8:
      {
        bins_value_ = 4096;
        break;
      }
      case 9:
      {
        bins_value_ = 8192;
        break;
      }
      case 10:
      {
        bins_value_ = 16394;
        break;
      }
      default:
      {
        bins_value_ = 256;
        break;
      }
    }
  });
  bins_selection_->setCurrentIndex(4);

  QPushButton *topic(new QPushButton("Topic"));
  topic->setToolTip("Select the image topic");
  connect(topic, &QPushButton::clicked, this, &HistogramPanel::topicSelectionSlot);

  buttons->addWidget(topic);
  buttons->addStretch(1);
  buttons->addWidget(start_stop_);
  buttons->addStretch(1);
  buttons->addWidget(refresh_label);
  buttons->addWidget(graph_refresh_frequency_);
  buttons->addStretch(1);
  buttons->addWidget(bins_selection_label);
  buttons->addWidget(bins_selection_);

  layout->addLayout(buttons);
  layout->addWidget(custom_plot_);
}

HistogramPanel::~HistogramPanel()
{
  graph_refresh_timer_->stop();
}

void HistogramPanel::updateChartSlot()
{
  std::lock_guard<std::mutex> lock(data_ticks_mutex_);

  if (grayscale_)
  {
    if (data_.size() != ticks_.size() || data_.empty())
      return;
    bars_->setData(ticks_, data_, true);
    bars_->setVisible(true);
    blue_channel_data_.clear();
    green_channel_data_.clear();
    red_channel_data_.clear();
    bars_blue_->setVisible(false);
    bars_green_->setVisible(false);
    bars_red_->setVisible(false);
  }
  else
  {
    if ((blue_channel_data_.size() != ticks_.size() || blue_channel_data_.empty()) ||
        (green_channel_data_.size() != ticks_.size() || green_channel_data_.empty()) ||
        (red_channel_data_.size() != ticks_.size() || red_channel_data_.empty()))
      return;
    bars_blue_->setData(ticks_, blue_channel_data_, true);
    bars_green_->setData(ticks_, green_channel_data_, true);
    bars_red_->setData(ticks_, red_channel_data_, true);
    bars_->setVisible(false);
    data_.clear();
    bars_blue_->setVisible(true);
    bars_green_->setVisible(true);
    bars_red_->setVisible(true);
  }

  custom_plot_->rescaleAxes();
  custom_plot_->yAxis->setRange(0, histogram_max_counter_);
  custom_plot_->xAxis->setRange(0, bins_value_);
  custom_plot_->replot();
}

void HistogramPanel::topicSelectionSlot()
{
  std::vector<std::string> allowed_topics;
  allowed_topics.emplace_back("sensor_msgs/Image");

  std::deque<std::shared_ptr<TopicData>> displayed_topics;
  SelectionTopics *topic_window = new SelectionTopics(nh_, displayed_topics, allowed_topics, true);

  if (topic_window->supported_topics_.empty())
  {
    Q_EMIT displayMessageBox("No supported topic", "Error with topics, no supported topics found.", "",
                             QMessageBox::Icon::Warning);
    return;
  }

  if (!topic_window->exec() || topic_window->displayed_topics_.empty())
    return;

  Q_EMIT subscribeToTopic(QString::fromStdString(topic_window->displayed_topics_.at(0)->topic_name_));
  Q_EMIT configChanged();
}

void HistogramPanel::subscribeToTopicSlot(const QString topic)
{
  if (topic.isEmpty())
    return;

  topic_ = topic;
  sub_ = nh_->subscribe(topic.toStdString(), 1, &HistogramPanel::imageCallback, this);

  custom_plot_->legend->setVisible(false);
  start_stop_->setEnabled(true);
  updating_ = true;
  start_stop_->click();
}

void HistogramPanel::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  histogram_max_counter_ = 0;
  std::lock_guard<std::mutex> lock(data_ticks_mutex_);

  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (const cv::Exception &e)
  {
    std::string error("Error converting the image: ");
    error += e.what();
    ROS_ERROR_STREAM_NAMED(getName().toStdString(), error);
    return;
  }

  grayscale_ = false;
  if (cv_image->image.channels() == 1)
    grayscale_ = true;

  std::vector<int> histogram_size = {bins_value_};

  ticks_.resize(bins_value_);
  for (int i(0); i < ticks_.size(); ++i)
    ticks_[i] = i;

  if (cv_image->image.channels() == 1) // Grayscale images
  {
    std::vector<cv::Mat> input_image = {cv_image->image};

    cv::Mat_<float> tmp_mat; // Only to initialize the output data structure histogram
    cv::OutputArray histogram(tmp_mat);

    data_.clear();

    if (cv_image->image.depth() == CV_8U) // 8 bits images
    {
      std::vector<float> ranges = {0, 256};
      cv::calcHist(input_image, {0}, cv::Mat(), histogram, histogram_size, ranges);
      for (int i(0); i < ticks_.size(); ++i)
      {
        data_.push_back(histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = histogram.getMatRef().at<float>(i);
      }
    }
    else if (cv_image->image.depth() == CV_16U) // 16 bits images
    {
      std::vector<float> ranges = {0, 65536};
      cv::calcHist(input_image, {0}, cv::Mat(), histogram, histogram_size, ranges);
      for (int i(0); i < ticks_.size(); ++i)
      {
        data_.push_back(histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = histogram.getMatRef().at<float>(i);
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(getName().toStdString(),
                             "Image format is not supported: only CV_8U and CV_16U images are supported");
      return;
    }
  }
  else if (cv_image->image.channels() == 3) // Color images
  {
    cv::Mat_<float> bgr[3];
    split(cv_image->image, bgr);
    std::vector<cv::Mat> input_image_b = {bgr[0]};
    std::vector<cv::Mat> input_image_g = {bgr[1]};
    std::vector<cv::Mat> input_image_r = {bgr[2]};

    cv::Mat_<float> tmp_mat_b; // Only to initialize the output data structure blue channel histogram, blue_histogram
    cv::Mat_<float> tmp_mat_g; // Only to initialize the output data structure green channel histogram, green_histogram
    cv::Mat_<float> tmp_mat_r; // Only to initialize the output data structure red channel histogram, red_histogram
    cv::OutputArray blue_histogram(tmp_mat_b);
    cv::OutputArray green_histogram(tmp_mat_g);
    cv::OutputArray red_histogram(tmp_mat_r);

    blue_channel_data_.clear();
    green_channel_data_.clear();
    red_channel_data_.clear();

    if (cv_image->image.depth() == CV_8U) //8 bits images
    {
      std::vector<float> ranges = {0, 256};
      cv::calcHist(input_image_b, {0}, cv::Mat(), blue_histogram, histogram_size, ranges);
      cv::calcHist(input_image_g, {0}, cv::Mat(), green_histogram, histogram_size, ranges);
      cv::calcHist(input_image_r, {0}, cv::Mat(), red_histogram, histogram_size, ranges);

      for (int i(0); i < ticks_.size(); ++i)
      {
        blue_channel_data_.push_back(blue_histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < blue_histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = blue_histogram.getMatRef().at<float>(i);
        green_channel_data_.push_back(green_histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < green_histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = green_histogram.getMatRef().at<float>(i);
        red_channel_data_.push_back(red_histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < red_histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = red_histogram.getMatRef().at<float>(i);
      }
    }
    else if (cv_image->image.depth() == CV_16U) //16 bits images
    {
      std::vector<float> ranges = {0, 65536};
      cv::calcHist(input_image_b, {0}, cv::Mat(), blue_histogram, histogram_size, ranges);
      cv::calcHist(input_image_g, {0}, cv::Mat(), green_histogram, histogram_size, ranges);
      cv::calcHist(input_image_r, {0}, cv::Mat(), red_histogram, histogram_size, ranges);

      for (int i(0); i < ticks_.size(); ++i)
      {
        blue_channel_data_.push_back(blue_histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < blue_histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = blue_histogram.getMatRef().at<float>(i);
        green_channel_data_.push_back(green_histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < green_histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = green_histogram.getMatRef().at<float>(i);
        red_channel_data_.push_back(red_histogram.getMatRef().at<float>(i));
        if (histogram_max_counter_ < red_histogram.getMatRef().at<float>(i))
          histogram_max_counter_ = red_histogram.getMatRef().at<float>(i);
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(getName().toStdString(),
                             "Image format is not supported: only CV_8U and CV_16U images are supported");
      return;
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(getName().toStdString(),
                           "Image depth is not 1 or 3 : this format is not supported. Only grayscale or color images are supported.");
    return;
  }
}

void HistogramPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);

  int tmp_int;
  QString tmp_str;

  if (config.mapGetString(objectName() + "_subscriber", &tmp_str))
    subscribeToTopicSlot(tmp_str);

  if (config.mapGetInt(objectName() + "_graph_refresh_frequency", &tmp_int))
    graph_refresh_frequency_->setCurrentIndex(tmp_int);

  if (config.mapGetInt(objectName() + "_bins_selection", &tmp_int))
    bins_selection_->setCurrentIndex(tmp_int);
}

void HistogramPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue(objectName() + "_subscriber", QString::fromStdString(sub_.getTopic()));
  config.mapSetValue(objectName() + "_graph_refresh_frequency", graph_refresh_frequency_->currentIndex());
  config.mapSetValue(objectName() + "_bins_selection", bins_selection_->currentIndex());
}

void HistogramPanel::displayMessageBoxHandler(const QString title,
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

}
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(graph_rviz_plugin::HistogramPanel, rviz::Panel)

