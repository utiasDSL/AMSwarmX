#ifndef GRAPH_RVIZ_PLUGIN_STYLE_HPP
#define GRAPH_RVIZ_PLUGIN_STYLE_HPP

#include <QStyle>
#include <QObject>
#include <QStringList>

namespace graph_rviz_plugin
{

class TopicStyle : public QObject
{
Q_OBJECT
public:
  TopicStyle();
  std::vector<std::pair<QString, Qt::PenStyle>> style_list_
      {
          std::make_pair<QString, Qt::PenStyle>("Solid", Qt::SolidLine),
          std::make_pair<QString, Qt::PenStyle>("Dash", Qt::DashLine),
          std::make_pair<QString, Qt::PenStyle>("Dot", Qt::DotLine),
          std::make_pair<QString, Qt::PenStyle>("DashDot", Qt::DashDotLine),
          std::make_pair<QString, Qt::PenStyle>("DashDotDot", Qt::DashDotDotLine)
      };
  Qt::PenStyle getStyleFromIndex(const int index) const;
  int getIndexFromStyle(const Qt::PenStyle style) const;
  QStringList getStylesStringList() const;
};

}

#endif
