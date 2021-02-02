#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <QTextCursor>
#include <QButtonGroup>

#include <ros/ros.h>
#include <vector>
#include <string>

#include "std_msgs/String.h"

namespace Ui {
class GUI;
}

class GUI : public QMainWindow
{
    Q_OBJECT

public:
  explicit GUI(QWidget *parent = 0);
  ~GUI();

  void init(ros::NodeHandle* n);

private:
  Ui::GUI *ui;
  ros::NodeHandle* n_;

  QButtonGroup switch_group_;

public slots:

signals:

};

#endif // GUI_H
