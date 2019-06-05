/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/pointcloud_publisher_qt/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pointcloud_publisher_qt {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv) {
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to
                     // on_...() callbacks in this class.
  QObject::connect(
      ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
      SLOT(aboutQt()));  // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing -
                                       // qt-designer should have this already
                                       // hardwired, but often loses it
                                       // (settings?).
  disableAllwidgets();
  /*********************
  ** Eable Gui
  **********************/
  QObject::connect(&qnode, SIGNAL(nodeReady()), this,
                   SLOT(enableAllwidgets()));
  /*********************
  ** Shut Down
  **********************/
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                   SLOT(updateLoggingView()));

  /*********************
  ** Auto Start
  **********************/
  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
  if (ui.checkbox_use_environment->isChecked()) {
    if (!qnode.init()) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if (!qnode.init(ui.line_edit_master->text().toStdString(),
                    ui.line_edit_host->text().toStdString())) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
    }
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if (state == 0) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() { ui.view_logging->scrollToBottom(); }

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(
      this, tr("About ..."),
      tr("<h2>PACKAGE_NAME pointcloud_publisher_qt </h2><p>Copyright Eric Zhang </p><p>An "
         "application for publishing point cloud data from .pcd files.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "pointcloud_publisher_qt");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url =
      settings.value("master_url", QString("http://192.168.1.2:11311/"))
          .toString();
  QString host_url =
      settings.value("host_url", QString("192.168.1.3")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
  }
  ui.spinBox_r->setValue(settings.value("rate", 10).toInt());
  ui.lineEdit_tn->setText(
      settings.value("topic_name", QString("/kinect/qhd/points")).toString());
  ui.lineEdit_fi->setText(
      settings.value("frame_id", QString("/kinect2_link")).toString());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "pointcloud_publisher_qt");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  settings.setValue("use_environment_variables",
                    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
                    QVariant(ui.checkbox_remember_settings->isChecked()));
  settings.setValue("rate", ui.spinBox_r->value());
  settings.setValue("topic_name", ui.lineEdit_tn->text());
  settings.setValue("frame_id", ui.lineEdit_fi->text());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace pointcloud_publisher_qt

void pointcloud_publisher_qt::MainWindow::on_pushButton_right_clicked(
    bool checked) {
  if (qnode.frame < qnode.total - 1)
    qnode.frame += 1;
  else
    qnode.frame = 0;
  qnode.log(qnode.Info, "Publish frame: " + std::to_string(qnode.frame));
}

void pointcloud_publisher_qt::MainWindow::on_pushButton_left_clicked(
    bool checked) {
  if (qnode.frame > 0)
    qnode.frame -= 1;
  else
    qnode.frame = 8;
  qnode.log(qnode.Info, "Publish frame: " + std::to_string(qnode.frame));
}

void pointcloud_publisher_qt::MainWindow::on_pushButton_loop_clicked(
    bool checked) {
  qnode.loop = !qnode.loop;
  qnode.log(qnode.Info, "Loop: " + std::to_string(qnode.loop));
}

void pointcloud_publisher_qt::MainWindow::on_pushButton_P_toggled(
    bool checked) {
  if(checked){
    qnode.publish = true;
    ui.pushButton_P->setText("Stop");
  }else{
    qnode.publish = false;
    ui.pushButton_P->setText("Publish");
  }
  qnode.log(qnode.Info, "Publish: " + std::to_string(qnode.publish));
}

void pointcloud_publisher_qt::MainWindow::on_spinBox_r_editingFinished() {
  int rate = ui.spinBox_r->value();
  ros::param::set("~rate", rate);
  qnode.log(qnode.Info, "Publish rate: " + std::to_string(rate));
}

void pointcloud_publisher_qt::MainWindow::on_lineEdit_tn_editingFinished() {
  std::string topic_name = ui.lineEdit_tn->text().toStdString();
  ros::param::set("~topic_name", topic_name);
  qnode.log(qnode.Info, "Topic Name: " + topic_name);
}

void pointcloud_publisher_qt::MainWindow::on_lineEdit_fi_editingFinished() {
  std::string frame_id = ui.lineEdit_fi->text().toStdString();
  ros::param::set("~frame_id", frame_id);
  qnode.log(qnode.Info, "Frame ID: " + frame_id);
}

void pointcloud_publisher_qt::MainWindow::disableAllwidgets(){
  ui.groupBox_log->setEnabled(false);
}

void pointcloud_publisher_qt::MainWindow::enableAllwidgets(){
  ui.groupBox_log->setEnabled(true);
  ui.pushButton_P->setChecked(true);
}









