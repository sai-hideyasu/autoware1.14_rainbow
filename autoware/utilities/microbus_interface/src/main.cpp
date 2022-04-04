#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "microbus_interface");
    ros::NodeHandle nh, private_nh("~");

    MainWindow w(nh, private_nh);
    w.setAttribute(Qt::WA_DeleteOnClose);
    w.show();

    //return a.exec();

    ros::Rate loop_rate(100);
    while (ros::ok() && w.isVisible()){
      ros::spinOnce();
      app.processEvents();
      w.window_updata();
      loop_rate.sleep();
    }
}
