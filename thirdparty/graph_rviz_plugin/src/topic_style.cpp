#include <graph_rviz_plugin/topic_style.hpp>

namespace graph_rviz_plugin
{

TopicStyle::TopicStyle()
{
}

Qt::PenStyle TopicStyle::getStyleFromIndex(const int index) const
{
  if ((unsigned) index >= style_list_.size())
    return style_list_.at(0).second;

  return style_list_.at(index).second;
}

int TopicStyle::getIndexFromStyle(const Qt::PenStyle style) const
{
  for (unsigned i(0); i < style_list_.size(); ++i)
  {
    if (style_list_.at(i).second == style)
      return i;
  }

  return 0;
}

QStringList TopicStyle::getStylesStringList() const
{
  QStringList list;

  for (const auto& pair : style_list_)
    list.push_back(pair.first);

  return list;
}

}
