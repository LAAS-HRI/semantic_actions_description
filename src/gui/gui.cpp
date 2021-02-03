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

    QObject::connect(ui->search_lineEdit, SIGNAL(textChanged(const QString&)), this, SLOT(SearchChangedSlot(const QString&)));

    QObject::connect(ui->tabWidget_lists, SIGNAL(currentChanged(int)), this, SLOT(listTabChangedSlot(int)));
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

void GUI::listTabChangedSlot(int index)
{
  if(index == 0)
    filterList(ui->search_lineEdit->text());
  else
    filterTree(ui->search_lineEdit->text());
}

void GUI::SearchChangedSlot(const QString& search_text)
{
  if(ui->tabWidget_lists->currentIndex() == 0)
    filterList(search_text);
  else
    filterTree(search_text);
}

void GUI::filterList(const QString& search_text)
{
  for(size_t i = 0; i < ui->listWidget_list->count(); i++)
  {
    auto item = ui->listWidget_list->item(i);
    if(item->text().contains(search_text,  Qt::CaseInsensitive))
      item->setHidden(false);
    else
      item->setHidden(true);
  }
}

void GUI::filterTree(const QString& search_text)
{
  for(size_t i = 0; i < ui->treeWidget_tree->topLevelItemCount(); i++)
  {
    auto item = ui->treeWidget_tree->topLevelItem(i);
    if(hiddenChildren(item, search_text) == false)
      item->setHidden(false);
    else if(item->text(0).contains(search_text,  Qt::CaseInsensitive))
      item->setHidden(false);
    else
      item->setHidden(false);
  }
}

bool GUI::hiddenChildren(QTreeWidgetItem* parent_item, const QString& search_text)
{
  bool hidden = true;
  for(size_t i = 0 ; i < parent_item->childCount(); i++)
  {
    auto item = parent_item->child(i);
    if(hiddenChildren(item, search_text) == false)
    {
      item->setHidden(false);
      hidden = false;
    }
    else if(item->text(0).contains(search_text,  Qt::CaseInsensitive))
    {
      item->setHidden(false);
      hidden = false;
    }
    else
      item->setHidden(true);
  }
  return hidden;
}
