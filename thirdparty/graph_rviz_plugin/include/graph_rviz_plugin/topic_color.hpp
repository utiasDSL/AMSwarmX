#ifndef GRAPH_RVIZ_PLUGIN_COLOR_HPP
#define GRAPH_RVIZ_PLUGIN_COLOR_HPP

#include <QColor>
#include <QObject>
#include <QStringList>

namespace graph_rviz_plugin
{

class TopicColor : public QObject
{
Q_OBJECT
public:
  TopicColor();
  std::vector<std::pair<QString, QColor>> colors_list_
      {
          std::make_pair<QString, QColor>("Black", Qt::GlobalColor::black),
          std::make_pair<QString, QColor>("Red", Qt::GlobalColor::red),
          std::make_pair<QString, QColor>("Cyan", Qt::GlobalColor::cyan),
          std::make_pair<QString, QColor>("Green", Qt::GlobalColor::green),
          std::make_pair<QString, QColor>("Blue", Qt::GlobalColor::blue),
          std::make_pair<QString, QColor>("Gray", Qt::GlobalColor::gray),
          std::make_pair<QString, QColor>("Magenta", Qt::GlobalColor::magenta),
          std::make_pair<QString, QColor>("Dark Red", Qt::GlobalColor::darkRed),
          std::make_pair<QString, QColor>("Dark Green", Qt::GlobalColor::darkGreen),
          std::make_pair<QString, QColor>("Dark Blue", Qt::GlobalColor::darkBlue),
          std::make_pair<QString, QColor>("Dark Cyan", Qt::GlobalColor::darkCyan),
          std::make_pair<QString, QColor>("Dark Magenta", Qt::GlobalColor::darkMagenta),
          std::make_pair<QString, QColor>("Yellow", Qt::GlobalColor::yellow),
          std::make_pair<QString, QColor>("Dark Yellow", Qt::GlobalColor::darkYellow),
          std::make_pair<QString, QColor>("Dark Gray", Qt::GlobalColor::darkGray),
          std::make_pair<QString, QColor>("Light Gray", Qt::GlobalColor::lightGray),
          std::make_pair<QString, QColor>("HSV", Qt::GlobalColor::transparent) // for HSV colormap
      };
  QColor getColorFromIndex(const int index) const;
  int getIndexFromColor(const QColor color) const;
  QStringList getColorsStringList() const;
};

}

#endif
