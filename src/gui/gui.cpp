#include "include/gui/gui.h"
#include "ui_gui.h"

#include <regex>

#define QUEU_SIZE 1000

GUI::GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GUI)
{
    ui->setupUi(this);
}

GUI::~GUI()
{
    delete ui;
}

void GUI::init(ros::NodeHandle* n)
{
  n_ = n;
}
