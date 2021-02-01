#include "include/gui/DarkStyle.h"
#include "include/gui/gui.h"

#include <QApplication>

#include <csignal>
#include <thread>

#include <ros/package.h>
#include <ros/ros.h>

void spinThread(bool* run)
{
  ros::Rate r(100);
  while(*run == true)
  {
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    a.setStyle(new DarkStyle);

    std::string path = ros::package::getPath("semantic_actions_description");
    path = path + "/resources/darkstyle/icon.ico";
    QIcon icon(QString::fromStdString(path));
    a.setWindowIcon(icon);

    GUI w;
    w.show();

    ros::init(argc, argv, "GUI");

    ros::NodeHandle n;
    bool run = true;

    w.init(&n);

    std::thread spin_thread(spinThread,&run);

    signal(SIGINT, SIG_DFL);
    auto a_exec = a.exec();

    run = false;
    spin_thread.join();

    return a_exec;
}


/*
  <customwidget>
   <class>QLineEditExtended</class>
   <extends>QLineEdit</extends>
   <header>include/ontologenius/graphical/ontoloGUI/QLineEditExtended.h</header>
  </customwidget>
*/