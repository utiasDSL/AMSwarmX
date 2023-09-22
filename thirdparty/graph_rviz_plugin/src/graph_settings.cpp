#include <graph_rviz_plugin/graph_settings.hpp>

namespace graph_rviz_plugin
{

GraphSettings::GraphSettings(std::deque<std::shared_ptr<TopicData>> displayed_topics, QDialog *) :
    displayed_topics_(displayed_topics)
{
  setWindowTitle("Graph settings");
  resize(600, 240);
  QVBoxLayout *configure_layout = new QVBoxLayout;
  setLayout(configure_layout);
  QStringList color_list = topic_color_.getColorsStringList();
  QStringList style_list = topic_style_.getStylesStringList();
  QTableWidget *main_table = new QTableWidget;
  main_table->setColumnCount(4);
  QStringList horiz_header_names = {"Display", "Color", "Thickness", "Style"};

  QStringList vertical_header_names;
  main_table->setHorizontalHeaderLabels(horiz_header_names);
  main_table->setRowCount(displayed_topics_.size());

  for (auto topic : displayed_topics_)
    vertical_header_names.push_back(QString::fromStdString(topic->topic_name_));

  main_table->setVerticalHeaderLabels(vertical_header_names);

  for (unsigned i(0); i < displayed_topics_.size(); ++i)
  {
    // Display
    QCheckBox *topic_checkbox = new QCheckBox;
    topic_buttons_.push_back(topic_checkbox);
    topic_checkbox->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    topic_checkbox->setChecked(true);

    if (displayed_topics_.at(i)->displayed_ == false)
      topic_checkbox->setChecked(false);

    main_table->setCellWidget(i, 0, topic_checkbox);

    // Color
    QComboBox *color_selection_combobox = new QComboBox;
    topic_color_combobox_.push_back(color_selection_combobox);
    color_selection_combobox->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    color_selection_combobox->addItems(color_list);
    color_selection_combobox->setCurrentIndex(topic_color_.getIndexFromColor(displayed_topics_.at(i)->color_));
    main_table->setCellWidget(i, 1, color_selection_combobox);

    // Thickness
    QSpinBox *thickness_spin_box = new QSpinBox;
    topic_spinbox_.push_back(thickness_spin_box);
    thickness_spin_box->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    thickness_spin_box->setRange(1, 10);
    thickness_spin_box->setValue(displayed_topics_.at(i)->thickness_);
    main_table->setCellWidget(i, 2, thickness_spin_box);

    // Line style
    QComboBox *style_selection_combobox = new QComboBox;
    topic_style_combobox_.push_back(style_selection_combobox);
    style_selection_combobox->setObjectName(QString::fromStdString(displayed_topics_.at(i)->topic_name_));
    style_selection_combobox->addItems(style_list);
    style_selection_combobox->setCurrentIndex(topic_style_.getIndexFromStyle(displayed_topics_.at(i)->style_));
    main_table->setCellWidget(i, 3, style_selection_combobox);
  }

  main_table->resizeColumnsToContents();
  main_table->resizeRowsToContents();

  configure_layout->addWidget(main_table);
  QDialogButtonBox *button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  configure_layout->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &GraphSettings::okClicked);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

GraphSettings::~GraphSettings()
{
}

void GraphSettings::okClicked()
{
  for (auto button : topic_buttons_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == button->objectName().toStdString()
          && button->isChecked())
        displayed_topics_.at(i)->displayed_ = true;

      if ((displayed_topics_.at(i)->topic_name_) == button->objectName().toStdString()
          && !button->isChecked())
        displayed_topics_.at(i)->displayed_ = false;
    }
  }

  for (auto spinbox : topic_spinbox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == spinbox->objectName().toStdString())
        displayed_topics_.at(i)->thickness_ = spinbox->value();
    }
  }

  for (auto combobox : topic_color_combobox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == combobox->objectName().toStdString())
      {
        const int index = combobox->currentIndex();
        displayed_topics_.at(i)->color_ = topic_color_.getColorFromIndex(index);
      }
    }
  }

  for (auto combobox : topic_style_combobox_)
  {
    for (unsigned i = 0; i < displayed_topics_.size(); i++)
    {
      if ((displayed_topics_.at(i)->topic_name_) == combobox->objectName().toStdString())
      {
        const int index = combobox->currentIndex();
        displayed_topics_.at(i)->style_ = topic_style_.getStyleFromIndex(index);
      }
    }
  }

  accept();
  return;
}

}
