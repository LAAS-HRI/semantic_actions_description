#include "include/gui/gui.h"
#include "ui_gui.h"

#include <regex>

#define QUEU_SIZE 1000

GUI::GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GUI)
{
    ui->setupUi(this);

    switch_group_.addButton(ui->actions_switch);
    switch_group_.addButton(ui->tasks_switch);
}

GUI::~GUI()
{
    delete ui;
}

void GUI::init(ros::NodeHandle* n)
{
  n_ = n;
  load();
}

void GUI::load()
{
  ActionReader reader(n_, "/actions_description");

  if(reader.load())
  {
    if(reader.getNbError() == 0)
    {
      items_ = reader.getActions();
      items_roots_ = reader.getRootActions();

      addItemsToList();
      addItemsToTree();
    }
    else
    {
      // print error
      std::cout << "errors" << std::endl;
    }
  }
  else
  {
    // print message
    std::cout << "no file" << std::endl;
  }
}

void GUI::addItemsToList()
{
  for(auto& item : items_)
  {
    ui->listWidget_list->addItem(new QListWidgetItem(QString::fromStdString(item.second->name)));
  }
}

void GUI::addItemsToTree()
{
  for(auto& root : items_roots_)
  {
    auto tree_item = new QTreeWidgetItem(QStringList(QString::fromStdString(root->name)));
    ui->treeWidget_tree->addTopLevelItem(tree_item);
    for(auto& child : root->children)
      addItemsToTree(child, tree_item);
  }
}

void GUI::addItemsToTree(action_t* item, QTreeWidgetItem* parent)
{
  auto tree_item = new QTreeWidgetItem(QStringList(QString::fromStdString(item->name)));
  parent->addChild(tree_item);
  parent->setExpanded(true);
  for(auto& child : item->children)
    addItemsToTree(child, tree_item);
}
