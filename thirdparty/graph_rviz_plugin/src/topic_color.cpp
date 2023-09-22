#include <graph_rviz_plugin/topic_color.hpp>

namespace graph_rviz_plugin
{

TopicColor::TopicColor()
{
}

QColor TopicColor::getColorFromIndex(const int index) const
{
  if ((unsigned) index >= colors_list_.size())
    return colors_list_.at(0).second;

  return colors_list_.at(index).second;
}

int TopicColor::getIndexFromColor(const QColor color) const
{
  for (unsigned i(0); i < colors_list_.size(); ++i)
  {
    if (colors_list_.at(i).second == color)
      return i;
  }

  return 0;
}

QStringList TopicColor::getColorsStringList() const
{
  QStringList list;

  for (auto pair : colors_list_)
    list.push_back(pair.first);

  return list;
}

}
