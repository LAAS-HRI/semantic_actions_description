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

    QObject::connect(ui->treeWidget_tree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(treeItemClickedSlot(QTreeWidgetItem*, int)));
    QObject::connect(ui->listWidget_list, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(listItemClickedSlot(QListWidgetItem*)));
    QObject::connect(ui->treeWidget_parents, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(treeItemClickedSlot(QTreeWidgetItem*, int)));
    QObject::connect(ui->listWidget_children, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(listItemClickedSlot(QListWidgetItem*)));

    QObject::connect(ui->actions_switch, SIGNAL(released()), this, SLOT(loadSlot()));
    QObject::connect(ui->tasks_switch, SIGNAL(released()), this, SLOT(loadSlot()));
    QObject::connect(ui->pushbutton_reload, SIGNAL(pressed()), this, SLOT(loadSlot()));
}

GUI::~GUI()
{
    delete ui;
}

void GUI::init(ros::NodeHandle* n)
{
  n_ = n;
  current_item_ = nullptr;
  load();
}

void GUI::load()
{
  std::string param_name;
  bool is_task = false;

  if(ui->actions_switch->checkState() == Qt::Checked)
  {
    param_name = "/actions_description";
    is_task = false;
  }
  else
  {
    param_name = "/tasks_description";
    is_task = true;
  }
  ActionReader reader(n_, param_name, is_task);
  current_item_ = nullptr;

  if(reader.load())
  {
    if(reader.getNbError() == 0)
    {
      items_ = reader.getActions();
      items_roots_ = reader.getRootActions();

      ui->listWidget_children->clear();
      ui->listWidget_list->clear();
      ui->treeWidget_parents->clear();
      ui->treeWidget_tree->clear();
      ui->textedit_decription->setHtml("");
      ui->search_lineEdit->setText("");

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
  ui->treeWidget_tree->resizeColumnToContents(0);
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

void GUI::treeItemClickedSlot(QTreeWidgetItem* item, int column)
{
  if(items_.find(item->text(0).toStdString()) == items_.end())
  {
    std::cout << "does not exist" << std::endl;
    return;
  }
  current_item_ = items_[item->text(0).toStdString()];

  if(current_item_ == nullptr)
  {
    std::cout << "current is null" << std::endl;
    return;
  }

  setDescription();
}

void GUI::listItemClickedSlot(QListWidgetItem* item)
{
  if(items_.find(item->text().toStdString()) == items_.end())
  {
    std::cout << "does not exist" << std::endl;
    return;
  }
  current_item_ = items_[item->text().toStdString()];

  if(current_item_ == nullptr)
  {
    std::cout << "current is null" << std::endl;
    return;
  }

  setDescription();
}

void GUI::setDescription()
{
  ui->listWidget_children->clear();
  for(auto child : current_item_->children)
    setInChildrenList(child);

  ui->treeWidget_parents->clear();
  setPrentInTree(current_item_->type);
  ui->treeWidget_parents->resizeColumnToContents(0);

  std::string text;
  text += "<h2>" + current_item_->name + "</h2>";
  text += "<p>" + current_item_->description + "</p>";

  std::pair<std::string, std::string> inherited_agent = getAgent();
  if(ui->actions_switch->checkState() == Qt::Checked)
    text += "<h3>Agent:</h3>";
  else
    text += "<h3>Agents:</h3>";
  if(current_item_->agent_type != "")
  {
    text += "<dl><dd>" + current_item_->agent_type;
    if(inherited_agent.first != "")
      text += " (overloaded)";
    text += "</dd></dl>";
  }
  else if(inherited_agent.first != "")
  {
    text += "<dl><dd>" + inherited_agent.second + " <i>(from " + inherited_agent.first + ")</i></dd></dl>";
  }

  std::map<std::string, std::pair<std::string, std::string>> inherited_params = getParameters();
  text += "<h3>Parameters:</h3>";
  if(current_item_->parameters.size() || inherited_params.size())
  {
    text += "<ul>";
    for(auto& param : current_item_->parameters)
    {
      text += "<li><b>" + param.second + "</b> : " + param.first;
      if(inherited_params.find(param.first) != inherited_params.end())
          text += " (overloaded)";
      text += "</li>";
    }

    for(auto& param : inherited_params)
    {
      if(current_item_->parameters.find(param.first) == current_item_->parameters.end())
      {
        text += "<li><b>" + param.second.second + "</b> : " + param.first + + " <i>(from " + param.second.first + ")</i></li>";
      }
    }

    text += "</ul>";
  }
  text += "<h3>Verbalization:</h3>";
  if(current_item_->verbalizations.size())
  {
    text += "<ul>";
    for(auto verbalization : current_item_->verbalizations)
      text += "<li>" + verbalization + "</li>";
    text += "</ul>";
  }

  std::string html = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">"
                  "<html><body style=\" font-family:'Noto Sans'; font-size:10pt; font-weight:400; font-style:normal;\">"
                  + text + "</body></html>";

  ui->textedit_decription->setHtml(QString::fromStdString(html));
}

std::pair<std::string, std::string> GUI::getAgent()
{
  action_t* item = current_item_->type;
  while(item != nullptr)
  {
    if(item->agent_type != "")
    {
      return {item->name, item->agent_type};
    }
    item = item->type;
  }

  return {"", ""};
}

std::map<std::string, std::pair<std::string, std::string>> GUI::getParameters()
{
  std::map<std::string, std::pair<std::string, std::string>> parameters;

  action_t* item = current_item_->type;
  while(item != nullptr)
  {
    for(auto& param : item->parameters)
    {
      if(parameters.find(param.first) == parameters.end())
        parameters[param.first] = {item->name, param.second};
    }
    item = item->type;
  }

  return parameters;
}

void GUI::setInChildrenList(action_t* item)
{
    ui->listWidget_children->addItem(new QListWidgetItem(QString::fromStdString(item->name)));
    for(auto child : item->children)
      setInChildrenList(child);
}

QTreeWidgetItem* GUI::setPrentInTree(action_t* item)
{
  if(item == nullptr)
  {
    return nullptr;
  }
  else if(item->type != nullptr)
  {
    QTreeWidgetItem* parent = setPrentInTree(item->type);

    auto tree_item = new QTreeWidgetItem(QStringList(QString::fromStdString(item->name)));
    parent->addChild(tree_item);
    parent->setExpanded(true);

    return tree_item;
  }
  else
  {
    auto tree_item = new QTreeWidgetItem(QStringList(QString::fromStdString(item->name)));
    ui->treeWidget_parents->addTopLevelItem(tree_item);
    return tree_item;
  }
}
