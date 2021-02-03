#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <QTextCursor>
#include <QButtonGroup>
#include <QTreeWidgetItem>
#include <QListWidgetItem>

#include <ros/ros.h>
#include <vector>
#include <string>

#include "std_msgs/String.h"

#include "reader/ActionReader.h"

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

  std::map<std::string, action_t*> items_;
  std::vector<action_t*> items_roots_;
  action_t* current_item_;

  void load();
  void addItemsToList();
  void addItemsToTree();
  void addItemsToTree(action_t* item, QTreeWidgetItem* parent);

  void filterList(const QString& search_text);
  void filterTree(const QString& search_text);
  bool hiddenChildren(QTreeWidgetItem* parent_item, const QString& search_text);

  void setDescription();
  void setInChildrenList(action_t* item);
  QTreeWidgetItem* setPrentInTree(action_t* item);

  std::pair<std::string, std::string> getAgent();
  std::map<std::string, std::pair<std::string, std::string>> getParameters();

public slots:

  void SearchChangedSlot(const QString&);
  void listTabChangedSlot(int index);

  void treeItemClickedSlot(QTreeWidgetItem* item, int column);
  void listItemClickedSlot(QListWidgetItem* item);

signals:

};

#endif // GUI_H
